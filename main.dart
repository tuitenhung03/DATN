import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'STM32 • ESP32 Dashboard',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.blue),
        useMaterial3: true,
      ),
      home: const DashboardPage(),
    );
  }
}

class SensorData {
  final int? t, h, gas, flame, alarm;
  SensorData({this.t, this.h, this.gas, this.flame, this.alarm});
  bool get hasValues => t != null || h != null || gas != null || flame != null || alarm != null;

  static int? _asInt(dynamic v) {
    if (v == null) return null;
    if (v is int) return v;
    if (v is double) return v.round();
    if (v is String) return int.tryParse(v);
    return null;
  }

  static SensorData? fromAnyJson(Map<String, dynamic> j) {
    // Dạng JSON trực tiếp
    if (j.containsKey('t') || j.containsKey('gas') || j.containsKey('flame') || j.containsKey('alarm')) {
      return SensorData(
        t: _asInt(j['t']),
        h: _asInt(j['h']),
        gas: _asInt(j['gas']),
        flame: _asInt(j['flame']),
        alarm: _asInt(j['alarm']),
      );
    }
    // Dạng bọc: {"device":"{\"t\":25,...}","ts":...}
    if (j['device'] is String) {
      try {
        final inner = jsonDecode(j['device']);
        if (inner is Map<String, dynamic>) return fromAnyJson(inner);
      } catch (_) {}
    }
    return null;
  }
}

class DashboardPage extends StatefulWidget {
  const DashboardPage({super.key});
  @override
  State<DashboardPage> createState() => _DashboardState();
}

class _DashboardState extends State<DashboardPage> {
  // ===== Persist keys =====
  static const _kBaseUrl = 'cfg.baseUrl';
  static const _kPolling = 'cfg.polling';
  static const _kIntervalMs = 'cfg.intervalMs';

  // ===== Controllers / state =====
  final TextEditingController _baseUrlCtrl = TextEditingController();
  final TextEditingController _intervalCtrl = TextEditingController(text: '1000');

  bool _polling = true;
  Timer? _timer;
  SensorData? _data;
  DateTime? _lastTs;
  String? _lastError;
  bool _fetching = false;

  @override
  void initState() {
    super.initState();
    _loadPrefs().then((_) {
      if (_polling) _startPolling();
    });
  }

  @override
  void dispose() {
    _timer?.cancel();
    _baseUrlCtrl.dispose();
    _intervalCtrl.dispose();
    super.dispose();
  }

  Future<void> _loadPrefs() async {
    final sp = await SharedPreferences.getInstance();
    _baseUrlCtrl.text = sp.getString(_kBaseUrl) ?? '';
    _polling = sp.getBool(_kPolling) ?? true;
    final ms = sp.getInt(_kIntervalMs) ?? 1000;
    _intervalCtrl.text = ms.toString();
    setState(() {});
  }

  Future<void> _savePrefs() async {
    final sp = await SharedPreferences.getInstance();
    await sp.setString(_kBaseUrl, _baseUrlCtrl.text.trim());
    await sp.setBool(_kPolling, _polling);
    final ms = int.tryParse(_intervalCtrl.text.trim()) ?? 1000;
    await sp.setInt(_kIntervalMs, ms);
  }

  void _startPolling() {
    _timer?.cancel();
    final ms = int.tryParse(_intervalCtrl.text.trim()) ?? 1000;
    _timer = Timer.periodic(Duration(milliseconds: ms), (_) => _fetchOnce());
    setState(() {});
  }

  void _stopPolling() {
    _timer?.cancel();
    _timer = null;
    setState(() {});
  }

  String _normalizeStatus(String base) {
    var b = base.trim();
    if (!b.startsWith('http://') && !b.startsWith('https://')) b = 'http://$b';
    if (b.endsWith('/')) b = b.substring(0, b.length - 1);
    if (!b.toLowerCase().endsWith('/status')) b = '$b/status';
    return b;
  }

  String _baseRoot(String base) {
    var b = base.trim();
    if (!b.startsWith('http://') && !b.startsWith('https://')) b = 'http://$b';
    if (b.endsWith('/')) b = b.substring(0, b.length - 1);
    if (b.toLowerCase().endsWith('/status')) b = b.substring(0, b.length - '/status'.length);
    return b;
  }

  Future<void> _sendServo(int value) async {
    final root = _baseRoot(_baseUrlCtrl.text);
    final url = '$root/servo?value=$value';
    try {
      final resp = await http.get(Uri.parse(url)).timeout(const Duration(seconds: 5));
      if (!mounted) return;
      if (resp.statusCode == 200) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text(value == 1 ? 'Servo OPEN' : 'Servo CLOSE')),
        );
      } else {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('Servo HTTP ${resp.statusCode}: ${resp.reasonPhrase ?? ""}')),
        );
      }
    } catch (e) {
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Servo error: $e')),
      );
    }
  }

  Future<void> _fetchOnce() async {
    final base = _baseUrlCtrl.text.trim();
    if (base.isEmpty) {
      setState(() => _lastError = 'Chưa nhập IP/URL ESP32 (vd: http://192.168.1.50)');
      return;
    }
    final url = _normalizeStatus(base);
    if (_fetching) return;
    _fetching = true;
    try {
      final resp = await http.get(Uri.parse(url)).timeout(const Duration(seconds: 5));
      if (resp.statusCode == 200) {
        final body = resp.body.trim();
        Map<String, dynamic>? jsonMap;
        try {
          jsonMap = jsonDecode(body);
        } catch (_) {
          try {
            // nếu ESP trả chuỗi STM32 thuần (không phải object), bọc lại
            jsonMap = jsonDecode('{"device":$body}');
          } catch (_) {}
        }
        final parsed = (jsonMap != null) ? SensorData.fromAnyJson(jsonMap) : null;
        setState(() {
          _data = parsed;
          _lastTs = DateTime.now();
          _lastError = parsed == null ? 'Payload không đúng định dạng mong đợi.' : null;
        });
      } else {
        setState(() => _lastError = 'HTTP ${resp.statusCode}: ${resp.reasonPhrase ?? ""}'.trim());
      }
    } on TimeoutException {
      setState(() => _lastError = 'Timeout khi GET /status');
    } catch (e) {
      setState(() => _lastError = e.toString());
    } finally {
      _fetching = false;
    }
  }

  Color _tileColor(SensorData? d) {
    if (d == null) return Colors.grey.shade200;
    if ((d.alarm ?? 0) == 1) return Colors.red.withOpacity(0.1);
    if ((d.flame ?? 1) == 0) return Colors.deepOrange.withOpacity(0.12);
    return Colors.green.withOpacity(0.08);
  }

  @override
  Widget build(BuildContext context) {
    final d = _data;
    final last = _lastTs != null ? _fmtTime(_lastTs!) : '—';
    final err = _lastError;

    return Scaffold(
      appBar: AppBar(
        title: const Text('STM32 • ESP32 • Flutter Dashboard'),
        actions: [
          IconButton(onPressed: _fetchOnce, icon: const Icon(Icons.refresh)),
          const SizedBox(width: 8),
        ],
      ),
      body: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 1000),
          child: ListView(
            padding: const EdgeInsets.all(16),
            children: [
              _buildConfigCard(),
              const SizedBox(height: 12),
              _buildStatusBar(last, err),
              const SizedBox(height: 12),
              _buildDataGrid(d),
              const SizedBox(height: 12),
              _buildServoButtons(),
              const SizedBox(height: 16),
              _buildLegend(),
              if (kIsWeb) const SizedBox(height: 12),
              if (kIsWeb) _buildCorsHint(),
            ],
          ),
        ),
      ),
      floatingActionButton: FloatingActionButton.extended(
        onPressed: () async {
          _polling = !_polling;
          await _savePrefs();
          if (_polling) _startPolling(); else _stopPolling();
          setState(() {});
        },
        icon: Icon(_polling ? Icons.pause_circle_filled : Icons.play_circle_fill),
        label: Text(_polling ? 'Stop Polling' : 'Start Polling'),
      ),
    );
  }

  Widget _buildConfigCard() {
    return Card(
      elevation: 1,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: LayoutBuilder(
          builder: (_, c) {
            final wide = c.maxWidth > 700;
            final rows = <Widget>[
              Flexible(
                flex: 3,
                child: TextField(
                  controller: _baseUrlCtrl,
                  decoration: const InputDecoration(
                    labelText: 'ESP32 URL (vd: http://192.168.1.50 hoặc http://192.168.1.50/status)',
                    prefixIcon: Icon(Icons.link),
                    border: OutlineInputBorder(),
                  ),
                ),
              ),
              const SizedBox(width: 12, height: 12),
              SizedBox(
                width: 180,
                child: TextField(
                  controller: _intervalCtrl,
                  keyboardType: TextInputType.number,
                  decoration: const InputDecoration(
                    labelText: 'Interval (ms)',
                    prefixIcon: Icon(Icons.timer),
                    border: OutlineInputBorder(),
                  ),
                ),
              ),
              const SizedBox(width: 12, height: 12),
              FilledButton.icon(
                onPressed: () async {
                  await _savePrefs();
                  if (_polling) _startPolling();
                  if (mounted) {
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('Đã lưu cấu hình')),
                    );
                  }
                },
                icon: const Icon(Icons.save),
                label: const Text('Lưu'),
              ),
            ];
            return wide
                ? Row(crossAxisAlignment: CrossAxisAlignment.start, children: rows)
                : Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: rows);
          },
        ),
      ),
    );
  }

  Widget _buildStatusBar(String last, String? err) {
    return Card(
      color: err == null ? Colors.blueGrey.shade50 : Colors.red.shade50,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
        child: Row(
          children: [
            Icon(err == null ? Icons.cloud_done : Icons.error_outline,
                color: err == null ? Colors.green : Colors.red),
            const SizedBox(width: 12),
            Expanded(
              child: Text(
                err == null ? 'Kết nối OK • Lần cập nhật: $last' : 'Lỗi: $err',
                style: TextStyle(
                  fontWeight: FontWeight.w600,
                  color: err == null ? Colors.black87 : Colors.red.shade800,
                ),
              ),
            ),
            if (_timer != null)
              Text('Polling: ${(_timer!.isActive) ? "ON" : "OFF"}',
                  style: const TextStyle(fontWeight: FontWeight.w500))
          ],
        ),
      ),
    );
  }

  Widget _buildDataGrid(SensorData? d) {
    return Card(
      color: _tileColor(d),
      elevation: 1,
      child: Padding(
        padding: const EdgeInsets.all(14),
        child: LayoutBuilder(
          builder: (_, cons) {
            final twoCols = cons.maxWidth > 540;
            final tiles = [
              _metricTile('Nhiệt độ', d?.t != null ? '${d!.t} °C' : '—', Icons.thermostat),
              _metricTile('Độ ẩm', d?.h != null ? '${d!.h} %' : '—', Icons.water_drop),
              _metricTile('GAS', d?.gas != null ? '${d!.gas} %' : '—', Icons.local_gas_station),
              _metricTile(
                'Flame',
                d?.flame != null ? (d!.flame == 0 ? 'PHÁT HIỆN LỬA (0)' : 'Không có lửa (1)') : '—',
                Icons.local_fire_department,
                emphasize: d?.flame == 0,
              ),
              _metricTile(
                'ALARM',
                d?.alarm != null ? (d!.alarm == 1 ? 'BẬT' : 'TẮT') : '—',
                Icons.alarm_on,
                emphasize: d?.alarm == 1,
              ),
            ];
            if (twoCols) {
              return Wrap(
                spacing: 12, runSpacing: 12,
                children: tiles.map((w) => SizedBox(width: (cons.maxWidth - 12) / 2, child: w)).toList(),
              );
            }
            return Column(
              children: tiles.map((w) => Padding(padding: const EdgeInsets.only(bottom: 12), child: w)).toList(),
            );
          },
        ),
      ),
    );
  }

  Widget _metricTile(String title, String value, IconData icon, {bool emphasize = false}) {
    final color = emphasize ? Colors.red : Colors.black87;
    return Container(
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: emphasize ? Colors.red : Colors.grey.shade300),
      ),
      child: Row(
        children: [
          Icon(icon, size: 28, color: color),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(title, style: TextStyle(fontSize: 14, color: Colors.grey.shade700, fontWeight: FontWeight.w600)),
                const SizedBox(height: 2),
                Text(value, style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: color)),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildServoButtons() {
    return Card(
      elevation: 1,
      child: Padding(
        padding: const EdgeInsets.all(14),
        child: Row(
          children: [
            Expanded(
              child: FilledButton.icon(
                onPressed: () => _sendServo(1),
                icon: const Icon(Icons.lock_open), // <-- sửa để tránh lỗi icon
                label: const Text('MỞ SERVO'),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: OutlinedButton.icon(
                onPressed: () => _sendServo(0),
                icon: const Icon(Icons.meeting_room),
                label: const Text('ĐÓNG SERVO'),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildLegend() {
    return Row(
      children: [
        _legendSwatch(Colors.green.withOpacity(0.08), 'Bình thường'),
        const SizedBox(width: 12),
        _legendSwatch(Colors.deepOrange.withOpacity(0.12), 'Flame = 0 (có lửa)'),
        const SizedBox(width: 12),
        _legendSwatch(Colors.red.withOpacity(0.10), 'ALARM = 1'),
      ],
    );
  }

  Widget _legendSwatch(Color c, String label) {
    return Row(
      children: [
        Container(width: 18, height: 18, decoration: BoxDecoration(color: c, borderRadius: BorderRadius.circular(4))),
        const SizedBox(width: 8),
        Text(label),
      ],
    );
  }

  Widget _buildCorsHint() {
    return Card(
      elevation: 0,
      color: Colors.amber.shade50,
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          children: [
            const Icon(Icons.info_outline),
            const SizedBox(width: 10),
            Expanded(
              child: Text(
                'Nếu chạy Flutter Web mà gọi bị chặn CORS: ESP32 đã bật CORS. Mobile/Desktop không bị CORS.',
                style: TextStyle(color: Colors.brown.shade800),
              ),
            ),
          ],
        ),
      ),
    );
  }

  String _fmtTime(DateTime dt) {
    final h = dt.hour.toString().padLeft(2, '0');
    final m = dt.minute.toString().padLeft(2, '0');
    final s = dt.second.toString().padLeft(2, '0');
    return '$h:$m:$s';
  }
}
