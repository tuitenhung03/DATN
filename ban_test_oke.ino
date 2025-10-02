#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Preferences.h>
#include <vector>

/* ===================== WiFi ===================== */
const char* WIFI_SSID = "SIUUUU";
const char* WIFI_PASS = "83868386";

/* ===================== UART2 (STM32 -> ESP32) ===================== */
static const int RXD2     = 16;       // STM32 TX -> ESP32 RX2
static const int TXD2     = 17;       // ESP32 TX2 -> STM32 RX (không bắt buộc)
static const uint32_t STM_BAUD = 115200;

/* ===================== SERVO ===================== */
static const int SERVO_PIN    = 15;
static const int SERVO_CLOSED = 0;
static const int SERVO_OPEN   = 90;
Servo gateServo;

// Thời gian mở khi có cảnh báo (nếu KHÔNG có field "servo")
static const uint32_t ALARM_OPEN_MS = 3000; // ms

// Trạng thái servo gần nhất (để tránh set lặp) & non-blocking hold
int  lastServoCmd     = -1;    // -1 unknown, 0 close, 1 open
bool servoHolding     = false;
unsigned long servoHoldUntil = 0;

/* ===================== LED cảnh báo ===================== */
#define LED_PIN 2

/* ===================== WebServer ===================== */
WebServer server(80);

/* ===================== Buffers JSON từ STM32 ===================== */
String lineBuf;   // tích lũy đến khi gặp '\n'
String lastJson = "{\"t\":-1,\"h\":-1,\"gas\":-1,\"flame\":1,\"alarm\":0}";
unsigned long lastOkMillis = 0;

/* ===================== RC522 (RFID) ===================== */
#define RC522_SS_PIN   5
#define RC522_RST_PIN  22
MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);

/* ====== Danh sách UID hợp lệ (đơn giản) ====== */
const char* ALLOWED_UIDS_SIMPLE[] = {
  "03FEBC13",
};
const size_t ALLOWED_SIMPLE_COUNT =
  sizeof(ALLOWED_UIDS_SIMPLE) / sizeof(ALLOWED_UIDS_SIMPLE[0]);

// Tránh lặp UID khi giữ thẻ
String lastUID = "";
unsigned long lastActionMs = 0;
const unsigned long SAME_CARD_COOLDOWN_MS = 1000; // 1s

/* ===================== Keypad 4x4 ===================== */
const byte ROWS = 4, COLS = 4;
char hexaKeys[ROWS][COLS] = {
  { '1','2','3','A' },
  { '4','5','6','B' },
  { '7','8','9','C' },
  { '*','0','#','D' }
};
byte rowPins[ROWS] = { 32, 33, 25, 26 }; // R1..R4
byte colPins[COLS] = { 27, 14, 12, 13 }; // C1..C4
Keypad keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

/* ===================== LCD I2C ===================== */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ===================== NVS (PIN & UIDs) ===================== */
Preferences prefs;
String PASSWORD = "1234";     // override nếu NVS có
const uint8_t MAX_LEN = 8;
const uint8_t MIN_LEN = 4;
String inputBuf = "";
uint8_t wrongCount = 0;          // sai PIN
const uint8_t MAX_WRONG = 3;
uint8_t cardWrongCount = 0;      // sai thẻ
const uint8_t MAX_CARD_WRONG = 3;
const uint32_t LOCK_MS = 10000;
uint32_t lockUntil = 0;

// Danh sách UIDs runtime (đọc/ghi NVS)
std::vector<String> allowedUIDs;
const size_t MAX_UIDS = 50;

// Mặc định (nếu NVS chưa có dữ liệu)
const char* ALLOWED_UIDS_DEFAULT[] = {
  "03FEBC13",
};
const size_t ALLOWED_DEFAULT_COUNT =
  sizeof(ALLOWED_UIDS_DEFAULT) / sizeof(ALLOWED_UIDS_DEFAULT[0]);

/* ===================== RFID+Keypad State machine ===================== */
enum Mode {
  MODE_ENTER_PIN,
  MODE_CHG_ASK_OLD,
  MODE_CHG_NEW,
  MODE_CHG_CONFIRM,
  MODE_ADD_WAIT_CARD,
  MODE_DEL_WAIT_CARD
};
Mode mode = MODE_ENTER_PIN;
String newPinTmp = "";

/* =====================================================
 * ================ TIỆN ÍCH DÙNG CHUNG ================
 * ===================================================== */
// Trả về ID ngắn của ESP32 (6 hex, lấy 24 bit cuối MAC)
String esp32ShortId() {
  uint64_t mac = ESP.getEfuseMac();        // MAC 48-bit
  uint32_t low = (uint32_t)(mac & 0xFFFFFF);
  char buf[9];
  snprintf(buf, sizeof(buf), "%06X", low); // 6 hex, IN HOA
  return String(buf);
}

bool looksLikeJsonObject(const String& s) {
  if (s.length() < 8) return false;
  if (s[0] != '{' || s[s.length() - 1] != '}') return false;
  if (s.indexOf("\"t\"") < 0 && s.indexOf("\"gas\"") < 0 &&
      s.indexOf("\"flame\"") < 0 && s.indexOf("\"alarm\"") < 0 &&
      s.indexOf("\"servo\"") < 0) return false;
  return true;
}

// rút số nguyên theo key từ JSON đơn giản (nhẹ, không ArduinoJson)
bool extractIntField(const String& json, const char* key, int& outVal) {
  String k = String("\"") + key + "\":";
  int p = json.indexOf(k);
  if (p < 0) return false;
  p += k.length();
  while (p < (int)json.length() && isspace((unsigned char)json[p])) p++;
  bool neg = false;
  if (p < (int)json.length() && json[p] == '-') { neg = true; p++; }
  long val = 0; bool any = false;
  while (p < (int)json.length() && isDigit((unsigned char)json[p])) {
    any = true; val = val * 10 + (json[p] - '0'); p++;
  }
  if (!any) return false;
  outVal = (int)(neg ? -val : val);
  return true;
}

String uidToString(const MFRC522::Uid &uid) {
  String s;
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) s += "0";
    s += String(uid.uidByte[i], HEX);
  }
  s.toUpperCase();
  return s;
}

bool isAllowedSimple(const String &uidHex) {
  for (size_t i = 0; i < ALLOWED_SIMPLE_COUNT; i++) {
    if (uidHex.equalsIgnoreCase(ALLOWED_UIDS_SIMPLE[i])) return true;
  }
  return false;
}

/* ===================== SERVO helpers ===================== */
void servoOpen()  { gateServo.write(SERVO_OPEN);   Serial.println("[SERVO] OPEN");  }
void servoClose() { gateServo.write(SERVO_CLOSED); Serial.println("[SERVO] CLOSE"); }

// Đặt servo 0/1 (ưu tiên field "servo" từ STM32), chống lặp, hủy hold nếu đang hold
void applyServoSet(int v) {
  v = (v != 0) ? 1 : 0;
  if (lastServoCmd == v) return;
  servoHolding = false; // huỷ hẹn đóng vì đã set cứng
  lastServoCmd = v;
  if (v) servoOpen(); else servoClose();
  Serial.print("[SERVO] set from JSON servo="); Serial.println(v);
}

// NON-BLOCKING: mở rồi tự đóng sau hold_ms
void servoOpenThenClose(uint32_t hold_ms) {
  gateServo.write(SERVO_OPEN);
  lastServoCmd = 1;
  servoHolding = true;
  servoHoldUntil = millis() + hold_ms;
  Serial.println("[SERVO] OPEN (non-block)");
}

// Kích servo theo CẠNH LÊN alarm (0->1). Chỉ dùng khi KHÔNG có field "servo"
void applyServoFromAlarmEdge(int alarmVal) {
  static int lastAlarm = -1;  // khởi tạo để bắt lần đầu
  if (alarmVal == 1 && lastAlarm != 1) {
    Serial.println("[ALARM] Rising edge -> servo open then close (non-block)");
    servoOpenThenClose(ALARM_OPEN_MS);
  }
  lastAlarm = alarmVal;
}

/* ===================== LCD helpers ===================== */
void lcdCenterPrint(const String &line1, const String &line2 = "") {
  lcd.clear();
  int c1 = max(0, (16 - (int)line1.length()) / 2);
  lcd.setCursor(c1, 0); lcd.print(line1.substring(0,16));
  if (line2.length()) {
    int c2 = max(0, (16 - (int)line2.length()) / 2);
    lcd.setCursor(c2, 1); lcd.print(line2.substring(0,16));
  }
}

void showInputMasked(const String &title = "Enter PIN:") {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(title.substring(0, 16));
  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < inputBuf.length() && i < 16; i++) lcd.print('*');
}

/* ===================== NVS: UIDs & PIN ===================== */
bool saveUIDsToNVS() {
  String csv;
  for (size_t i = 0; i < allowedUIDs.size(); i++) {
    if (i) csv += ",";
    csv += allowedUIDs[i];
  }
  prefs.begin("lock", false);
  bool ok = prefs.putString("uids", csv) > 0 || (csv.length() == 0);
  prefs.end();
  return ok;
}

void loadUIDsFromNVS() {
  allowedUIDs.clear();
  prefs.begin("lock", true);
  String csv = prefs.getString("uids", "");
  prefs.end();

  if (csv.length() == 0) {
    for (size_t i = 0; i < ALLOWED_DEFAULT_COUNT; i++) {
      allowedUIDs.push_back(String(ALLOWED_UIDS_DEFAULT[i]));
    }
    saveUIDsToNVS();
  } else {
    int start = 0;
    while (start < (int)csv.length()) {
      int comma = csv.indexOf(',', start);
      if (comma < 0) comma = csv.length();
      String tok = csv.substring(start, comma);
      tok.trim();
      tok.toUpperCase();
      if (tok.length() > 0) allowedUIDs.push_back(tok);
      start = comma + 1;
    }
  }
}

bool isAllowedUID_NVS(const String &uidHex) {
  for (auto &u : allowedUIDs) if (uidHex.equalsIgnoreCase(u)) return true;
  return false;
}

bool addUID(const String &uidHex) {
  if (uidHex.length() == 0) return false;
  if (isAllowedUID_NVS(uidHex)) return false;
  if (allowedUIDs.size() >= MAX_UIDS) return false;
  allowedUIDs.push_back(uidHex);
  return saveUIDsToNVS();
}

bool removeUID(const String &uidHex) {
  for (size_t i = 0; i < allowedUIDs.size(); i++) {
    if (uidHex.equalsIgnoreCase(allowedUIDs[i])) {
      allowedUIDs.erase(allowedUIDs.begin() + i);
      return saveUIDsToNVS();
    }
  }
  return false;
}

void loadPassword() {
  prefs.begin("lock", true);
  String saved = prefs.getString("pin", "");
  prefs.end();
  if (saved.length() >= MIN_LEN && saved.length() <= MAX_LEN) {
    PASSWORD = saved;
  }
}

bool savePassword(const String &pin) {
  if (pin.length() < MIN_LEN || pin.length() > MAX_LEN) return false;
  prefs.begin("lock", false);
  bool ok = prefs.putString("pin", pin) > 0;
  prefs.end();
  if (ok) PASSWORD = pin;
  return ok;
}

/* ===================== Lock helper ===================== */
void triggerLock(const char* reason) {
  lockUntil = millis() + LOCK_MS;
  wrongCount = 0;
  cardWrongCount = 0;
  lcdCenterPrint("LOCKED", String(LOCK_MS / 1000) + "s");
  Serial.print("Locked: "); Serial.println(reason);
}

/* ===================== Submit PIN theo mode ===================== */
void handleSubmit() {
  if (mode == MODE_ENTER_PIN) {
    if (inputBuf == PASSWORD) {
      wrongCount = 0; cardWrongCount = 0; lockUntil = 0;
      lcdCenterPrint("Correct!", "Opening...");
      servoOpenThenClose(3000); // non-block
      lcdCenterPrint("Door Closed", "Enter PIN:");
      inputBuf = "";
      delay(800);
      showInputMasked();
    } else {
      wrongCount++;
      lcdCenterPrint("Wrong PIN!", String("Left: ") + String(MAX_WRONG - wrongCount));
      inputBuf = "";
      delay(1000);
      if (wrongCount >= MAX_WRONG) {
        triggerLock("wrong PIN");
      } else {
        showInputMasked();
      }
    }
    return;
  }

  if (mode == MODE_CHG_ASK_OLD) {
    if (inputBuf == PASSWORD) {
      mode = MODE_CHG_NEW; inputBuf = "";
      showInputMasked("New PIN:");
    } else {
      lcdCenterPrint("Old PIN Wrong", "");
      delay(800); mode = MODE_ENTER_PIN; inputBuf = ""; showInputMasked();
    }
    return;
  }

  if (mode == MODE_CHG_NEW) {
    if (inputBuf.length() >= MIN_LEN && inputBuf.length() <= MAX_LEN) {
      newPinTmp = inputBuf; inputBuf = "";
      mode = MODE_CHG_CONFIRM; showInputMasked("Confirm:");
    } else {
      lcdCenterPrint("Len 4..8", "Try again");
      delay(800); inputBuf = ""; showInputMasked("New PIN:");
    }
    return;
  }

  if (mode == MODE_CHG_CONFIRM) {
    if (inputBuf == newPinTmp) {
      if (savePassword(newPinTmp)) lcdCenterPrint("PIN Updated", "");
      else lcdCenterPrint("Save Fail", "");
      delay(1000);
      mode = MODE_ENTER_PIN; inputBuf = ""; newPinTmp = ""; showInputMasked();
    } else {
      lcdCenterPrint("Mismatch", "");
      delay(800);
      mode = MODE_CHG_NEW; inputBuf = ""; newPinTmp = ""; showInputMasked("New PIN:");
    }
    return;
  }
}

/* ===================== WebServer Handlers ===================== */
void sendCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Access-Control-Allow-Methods", "GET,OPTIONS");
}

void handleOptions() { sendCORS(); server.send(204); }

void handleStatus() {
  sendCORS();
  server.send(200, "application/json", lastJson); // luôn trả JSON thuần
}

void handleStatus2() {
  // Bọc {"device":"<json string escaped>","ts":...}
  String esc; esc.reserve(lastJson.length() * 2);
  for (char c : lastJson) {
    if (c == '\"') esc += "\\\"";
    else if (c == '\\') esc += "\\\\";
    else esc += c;
  }
  String wrapped = String("{\"device\":\"") + esc + "\",\"ts\":" + String(millis()) + "}";
  sendCORS();
  server.send(200, "application/json", wrapped);
}

void handleServo() {
  sendCORS();
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int v = server.arg("value").toInt();   // 0 hoặc 1
  if (v != 0 && v != 1) { server.send(400, "text/plain", "value must be 0 or 1"); return; }

  // Manual set → ưu tiên & huỷ hold
  applyServoSet(v);
  server.send(200, "text/plain", "OK");
}

/* =====================================================
 * ========================= SETUP =====================
 * ===================================================== */
void setup() {
  Serial.begin(115200);
  Serial2.begin(STM_BAUD, SERIAL_8N1, RXD2, TXD2);

  // Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2400);
  gateServo.write(SERVO_CLOSED);
  lastServoCmd = 0;

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  Serial.print("OK, IP: "); Serial.println(WiFi.localIP());

  // Hiển thị ID + IP lên LCD trong 1.5s
  Wire.begin(21, 4);
  lcd.init(); lcd.backlight();
  {
    String id = esp32ShortId();
    String ip = WiFi.localIP().toString();
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("ID: "); lcd.print(id);
    lcd.setCursor(0, 1);
    if (ip.length() <= 16) lcd.print(ip);
    else                   lcd.print(ip.substring(0,16));
    delay(1500);
  }

  // Web routes
  server.on("/status",  HTTP_GET,     handleStatus);
  server.on("/status2", HTTP_GET,     handleStatus2);
  server.on("/servo",   HTTP_GET,     handleServo);
  server.onNotFound([](){ sendCORS(); server.send(404, "text/plain", "Not Found"); });
  server.on("/status",  HTTP_OPTIONS, handleOptions);
  server.on("/status2", HTTP_OPTIONS, handleOptions);
  server.on("/servo",   HTTP_OPTIONS, handleOptions);
  server.begin();

  // NVS
  loadPassword();
  loadUIDsFromNVS();

  // RFID
  SPI.begin();  // VSPI default: SCK18 MISO19 MOSI23
  mfrc522.PCD_Init();
  delay(50);
  Serial.println(F("RC522 ready. Quet the de doc UID..."));

  lcdCenterPrint("Ready", "Scan or PIN");
  delay(800);
  showInputMasked("Enter PIN:");
}

/* =====================================================
 * ========================== LOOP =====================
 * ===================================================== */
void loop() {
  // ---- đọc UART2 theo từng dòng (JSON từ STM32) ----
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;   // bỏ CR
    if (c == '\n') {
      String s = lineBuf; lineBuf = ""; s.trim();
      if (s.length() > 0) {
        if (looksLikeJsonObject(s)) {
          lastJson = s; lastOkMillis = millis();
          // ƯU TIÊN 1: field "servo"
          int servoVal;
          if (extractIntField(lastJson, "servo", servoVal)) {
            applyServoSet(servoVal);
          } else {
            // ƯU TIÊN 2: cạnh lên "alarm"
            int alarmVal;
            if (extractIntField(lastJson, "alarm", alarmVal))
              applyServoFromAlarmEdge(alarmVal);
            else
              applyServoFromAlarmEdge(0);
          }
        } else {
          // bỏ qua dòng không phải JSON
        }
      }
    } else {
      lineBuf += c;
      if (lineBuf.length() > 512) lineBuf.remove(0, lineBuf.length() - 256); // chống tràn
    }
  }

  // ====== LOCKED UI ======
  if (lockUntil > 0) {
    if ((int32_t)(millis() - lockUntil) < 0) {
      uint32_t remain = (lockUntil - millis() + 999) / 1000;
      lcd.setCursor(0, 0); lcd.print("LOCKED        ");
      lcd.setCursor(0, 1); lcd.print("Wait "); lcd.print(remain); lcd.print("s     ");
      server.handleClient();
      // KHÔNG return; để vẫn xử lý non-blocking close ở dưới
    } else {
      lockUntil = 0;
      wrongCount = 0; cardWrongCount = 0; inputBuf = "";
      mode = MODE_ENTER_PIN;
      lcdCenterPrint("Unlocked", "Enter PIN:");
      delay(800);
      showInputMasked("Enter PIN:");
    }
  }

  // ====== RFID xử lý ======
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    String uidHex = uidToString(mfrc522.uid);
    unsigned long now = millis();
    bool cooldownOK = !(uidHex == lastUID && (now - lastActionMs) < SAME_CARD_COOLDOWN_MS);
    if (mode == MODE_ADD_WAIT_CARD || mode == MODE_DEL_WAIT_CARD) cooldownOK = true;

    if (cooldownOK) {
      lastUID = uidHex;
      lastActionMs = now;

      if (mode == MODE_ADD_WAIT_CARD) {
        if (addUID(uidHex)) lcdCenterPrint("Added", uidHex.substring(0,8));
        else lcdCenterPrint("Add Fail", "");
        mode = MODE_ENTER_PIN; delay(1200); showInputMasked("Enter PIN:");
      }
      else if (mode == MODE_DEL_WAIT_CARD) {
        if (removeUID(uidHex)) lcdCenterPrint("Deleted", uidHex.substring(0,8));
        else lcdCenterPrint("Del Fail", "");
        mode = MODE_ENTER_PIN; delay(1200); showInputMasked("Enter PIN:");
      }
      else {
        bool okCard = isAllowedUID_NVS(uidHex) || isAllowedSimple(uidHex);
        if (okCard) {
          wrongCount = 0; cardWrongCount = 0; lockUntil = 0;
          lcdCenterPrint("RFID OK", "Opening...");
          servoOpenThenClose(3000); // NON-BLOCK
          lcdCenterPrint("Door Closed", "Scan/PIN");
          delay(800); showInputMasked("Enter PIN:");
        } else {
          cardWrongCount++;
          lcdCenterPrint("RFID WRONG", String("Left: ") + String(MAX_CARD_WRONG - cardWrongCount));
          if (cardWrongCount >= MAX_CARD_WRONG) triggerLock("wrong card");
          else showInputMasked("Enter PIN:");
        }
      }
    }
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }

  // ====== Keypad xử lý ======
  char key = keypad.getKey();
  if (key) {
    if (key == 'B' && mode == MODE_ENTER_PIN) {
      mode = MODE_ADD_WAIT_CARD; lcdCenterPrint("Add Card", "Scan now");
    }
    else if (key == 'C' && mode == MODE_ENTER_PIN) {
      mode = MODE_DEL_WAIT_CARD; lcdCenterPrint("Del Card", "Scan now");
    }
    else if (key == 'A' && mode == MODE_ENTER_PIN) {
      mode = MODE_CHG_ASK_OLD; inputBuf = ""; showInputMasked("Old PIN:");
    }
    else if (key == '#') {
      handleSubmit();
    }
    else if (key >= '0' && key <= '9' && inputBuf.length() < MAX_LEN) {
      inputBuf += key;
      showInputMasked(mode == MODE_ENTER_PIN ? "Enter PIN:" :
                      (mode == MODE_CHG_ASK_OLD ? "Old PIN:" :
                       (mode == MODE_CHG_NEW ? "New PIN:" : "Confirm:")));
    }
    else if (key == '*') {
      if (inputBuf.length() > 0) {
        inputBuf.remove(inputBuf.length() - 1);
        showInputMasked(mode == MODE_ENTER_PIN ? "Enter PIN:" :
                        (mode == MODE_CHG_ASK_OLD ? "Old PIN:" :
                         (mode == MODE_CHG_NEW ? "New PIN:" : "Confirm:")));
      }
    }
  }

  // WebServer
  server.handleClient();

  // ====== NON-BLOCKING CLOSE cho servo ======
  if (servoHolding && (long)(millis() - servoHoldUntil) >= 0) {
    gateServo.write(SERVO_CLOSED);
    lastServoCmd = 0;
    servoHolding = false;
    Serial.println("[SERVO] CLOSE (non-block)");
  }
}
