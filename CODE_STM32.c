/* ================== STM32F103C8T6: DHT11 + MQ-5 + Flame IR + LCD I2C + UART JSON
 *            + Rain DO + LED PC13 + Servo (TIM3 PWM 50Hz) ==================
 * - Toolchain: Keil / CubeIDE (CMSIS only, NO HAL)
 * - Clock: HSI 8MHz (default)
 * - UART1:  PA9 (TX) -> ESP RX, 115200
 * - I2C1:   PB6=SCL, PB7=SDA -> LCD I2C (PCF8574 @0x27 or 0x3F)
 * - DHT11:  PA3
 * - MQ-5:   PA2 (ADC1_IN2)
 * - Flame:  PA1 (DO, active-low: 0 = fire detected)
 * - Buzzer: PB5
 * - Rain:   PA0 (DO, usually active-low when wet)
 * - LED:    PC13 (active-low on BluePill board) -> ON when raining
 * - Servo:  PA6 -> TIM3_CH1 (PWM 50Hz, 1–2ms for 0–180°)
 * - Fan:    PB0 -> bật khi alarm=1, tắt khi alarm=0
 * ========================================================================== */
#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ===== Config thresholds ===== */
#define TH_TEMP_C          34
#define TH_GAS_PCT         50        // compare by % (0..100)
#define TH_GAS_mV          1650      // ~50% of 3.3V
#define GAS_COMPARE_BY_mV  0         // 0: by %, 1: by mV

/* ===== Pin mapping ===== */
#define FLAME_PORT   GPIOA
#define FLAME_PIN    1               // PA1 DO flame (0 = fire)

#define DHT_PORT     GPIOA
#define DHT_PIN      3               // PA3 DHT11

#define BUZZ_PORT    GPIOB
#define BUZZ_PIN     5               // PB5 buzzer

#define FAN_PORT     GPIOB
#define FAN_PIN      0               // PB0 fan (output PP)

#define MQ2_ADC_CH   2               // PA2 -> ADC1_IN2

#define I2C_LCD_ADDR 0x27            // PCF8574 (đổi 0x3F nếu cần)
#define I2C_PORT     GPIOB
#define I2C_SCL_PIN  6               // PB6
#define I2C_SDA_PIN  7               // PB7

#define RAIN_PORT    GPIOA
#define RAIN_PIN     0u              // PA0 (Rain DO)
#define LED_PORT     GPIOC
#define LED_PIN      13u             // PC13 (active-low)
#define SERVO_PORT   GPIOA
#define SERVO_PIN    6u              // PA6 -> TIM3_CH1
#define RAIN_ACTIVE_LOW   1          // 1: DO=0 khi ướt (đa số module)

/* ===== DWT timing ===== */
static void DWT_Init(void){
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
#define CYCLES_PER_US   (SystemCoreClock/1000000U)
#define CYCLES_US(us)   ((us) * CYCLES_PER_US)

static inline void HAL_DelayUs(uint32_t us){
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = CYCLES_US(us);
  while ((DWT->CYCCNT - start) < cycles);
}
static inline void delay_ms_dwt(uint32_t ms){
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = (SystemCoreClock/1000U) * ms;
  while ((DWT->CYCCNT - start) < cycles);
}

/* ===== I2C1 (100kHz) ===== */
static void i2c1_init(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  /* PB6/PB7: AF open-drain 50MHz */
  I2C_PORT->CRL &= ~((0xF<<24)|(0xF<<28));
  I2C_PORT->CRL |=  (0xF<<24) | (0xF<<28);

  I2C1->CR1 = I2C_CR1_SWRST; I2C1->CR1 = 0;

  uint32_t pclk1_mhz = SystemCoreClock/1000000U; if(pclk1_mhz>50) pclk1_mhz=36;
  I2C1->CR2   = pclk1_mhz;
  I2C1->CCR   = ((pclk1_mhz*1000000U) / (2*100000U)); if(I2C1->CCR<4) I2C1->CCR=4;
  I2C1->TRISE = pclk1_mhz + 1;
  I2C1->CR1  |= I2C_CR1_PE;
}
static void i2c1_start(uint8_t addr_wr){
  I2C1->CR1 |= I2C_CR1_START;
  while(!(I2C1->SR1 & I2C_SR1_SB));
  (void)I2C1->SR1; I2C1->DR = addr_wr;
  while(!(I2C1->SR1 & I2C_SR1_ADDR));
  (void)I2C1->SR1; (void)I2C1->SR2;
}
static void i2c1_stop(void){ I2C1->CR1 |= I2C_CR1_STOP; }
static void i2c1_write(uint8_t data){
  while(!(I2C1->SR1 & I2C_SR1_TXE)); I2C1->DR=data;
  while(!(I2C1->SR1 & I2C_SR1_BTF));
}

/* ===== LCD I2C (PCF8574) ===== */
#define LCD_BACKLIGHT 0x08
#define LCD_EN        0x04
#define LCD_RS        0x01
static uint8_t lcd_back = LCD_BACKLIGHT;

static void lcd_i2c_send(uint8_t d){ i2c1_start((I2C_LCD_ADDR<<1)|0); i2c1_write(d | lcd_back); i2c1_stop(); }
static void lcd_pulse(uint8_t d){
  lcd_i2c_send(d|LCD_EN); HAL_DelayUs(1);
  lcd_i2c_send(d&~LCD_EN); HAL_DelayUs(50);
}
static void lcd_write4(uint8_t n, uint8_t rs){ uint8_t d=(n&0xF)<<4; if(rs)d|=LCD_RS; lcd_pulse(d); }
static void lcd_cmd(uint8_t c){ lcd_write4(c>>4,0); lcd_write4(c,0); HAL_DelayUs(40); }
static void lcd_data(uint8_t c){ lcd_write4(c>>4,1); lcd_write4(c,1); HAL_DelayUs(40); }

static void lcd_init(void){
  delay_ms_dwt(40);
  lcd_write4(0x03,0); delay_ms_dwt(5);
  lcd_write4(0x03,0); HAL_DelayUs(150);
  lcd_write4(0x03,0); HAL_DelayUs(150);
  lcd_write4(0x02,0);
  lcd_cmd(0x28); lcd_cmd(0x08);
  lcd_cmd(0x01); delay_ms_dwt(2);
  lcd_cmd(0x06); lcd_cmd(0x0C);
}
static void lcd_clear(void){ lcd_cmd(0x01); delay_ms_dwt(2); }
static void lcd_set_cursor(uint8_t col,uint8_t row){ static const uint8_t rowAddr[2]={0,0x40}; lcd_cmd(0x80|(rowAddr[row]+col)); }
static void lcd_print(const char*s){ while(*s) lcd_data((uint8_t)*s++); }
static void lcd_print_pad(const char*s){
  char buf[17]; memset(buf,' ',16); buf[16]=0;
  size_t n=strlen(s); if(n>16)n=16; memcpy(buf,s,n);
  lcd_print(buf);
}

/* ===== ADC1 (MQ-5) ===== */
static void adc1_init(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
  GPIOA->CRL &= ~(0xF << (MQ2_ADC_CH*4));  // PA2 analog
  ADC1->CR2 |= ADC_CR2_ADON; delay_ms_dwt(1);
  ADC1->CR2 |= ADC_CR2_CAL; while(ADC1->CR2 & ADC_CR2_CAL);
}
static uint16_t adc1_read(uint8_t ch){
  ADC1->SQR3 = ch;
  ADC1->SMPR2 &= ~(0x7<<(ch*3));
  ADC1->SMPR2 |=  (0x4<<(ch*3));      // 55.5 cycles
  ADC1->CR2 |= ADC_CR2_ADON;
  while(!(ADC1->SR & ADC_SR_EOC));
  return ADC1->DR;
}
static uint8_t readGasPercent(void){
  uint16_t raw = adc1_read(MQ2_ADC_CH);
  return (uint8_t)((raw*100)/4095);
}
static uint32_t readGasMilliVolt(void){
  uint16_t raw = adc1_read(MQ2_ADC_CH);
  return (uint32_t)raw * 3300U / 4095U;
}

/* ===== UART1 (PA9/PA10) 115200@8MHz ===== */
static void usart1_init(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
  GPIOA->CRH &= ~((0xF<<4)|(0xF<<8));
  GPIOA->CRH |=  (0xB<<4) | (0x4<<8);   // PA9 AF-PP 50MHz; PA10 input floating
  USART1->CR1 = 0; USART1->CR2 = 0; USART1->CR3 = 0;
  USART1->BRR = 0x045;                 // 115200 @ 8MHz
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}
static void uart1_write_char(char c){ while(!(USART1->SR & USART_SR_TXE)); USART1->DR = (uint16_t)c; }
static void uart1_write(const char* s){ while(*s){ uart1_write_char(*s++); } }

/* ===== Flame (PA1) & Buzzer (PB5) & Fan (PB0) ===== */
static void flame_init_input_pullup(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIOA->CRL &= ~(0xF << (FLAME_PIN*4));
  GPIOA->CRL |=  (0x8 << (FLAME_PIN*4));  // input PU/PD
  GPIOA->BSRR  = (1U<<FLAME_PIN);         // pull-up
}
static inline uint8_t flame_read(void){ return (FLAME_PORT->IDR >> FLAME_PIN) & 1U; }

static void buzzer_init(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  GPIOB->CRL &= ~(0xF << (BUZZ_PIN*4));
  GPIOB->CRL |=  (0x2 << (BUZZ_PIN*4));   // MODE=10 (2MHz), CNF=00 (PP)
  BUZZ_PORT->BRR = (1U<<BUZZ_PIN);        // off
}
static inline void buzz_on(void){  BUZZ_PORT->BSRR = (1U<<BUZZ_PIN); }
static inline void buzz_off(void){ BUZZ_PORT->BRR  = (1U<<BUZZ_PIN); }

/* ---- Fan PB0 ---- */
static void fan_init(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  GPIOB->CRL &= ~(0xF << (FAN_PIN*4));
  GPIOB->CRL |=  (0x2 << (FAN_PIN*4));    // MODE=10 (2MHz), CNF=00 (PP)
  FAN_PORT->BRR = (1U<<FAN_PIN);          // OFF mặc định
}
static inline void fan_on(void){  FAN_PORT->BSRR = (1U<<FAN_PIN); }
static inline void fan_off(void){ FAN_PORT->BRR  = (1U<<FAN_PIN); }

/* ===== DHT11 (PA3) ===== */
static void dht_pin_out(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  DHT_PORT->CRL &= ~(0xF << (DHT_PIN*4));
  DHT_PORT->CRL |=  (0x2 << (DHT_PIN*4)); // 2MHz push-pull
}
static void dht_pin_in_pu(void){
  DHT_PORT->CRL &= ~(0xF << (DHT_PIN*4));
  DHT_PORT->CRL |=  (0x8 << (DHT_PIN*4)); // input PU/PD
  DHT_PORT->BSRR = (1U<<DHT_PIN);         // pull-up
}
static inline int wait_level(uint32_t level, uint32_t timeout_us){
  uint32_t start = DWT->CYCCNT;
  while( ((DHT_PORT->IDR >> DHT_PIN) & 1u) != (level & 1u) ){
    if ((DWT->CYCCNT - start) > CYCLES_US(timeout_us)) return -1;
  }
  return 0;
}
static int dht11_read(uint8_t* hum, uint8_t* temp){
  uint8_t d[5] = {0};

  /* Start signal */
  dht_pin_out();
  DHT_PORT->BRR  = (1U<<DHT_PIN);        // >=18ms
  delay_ms_dwt(18);
  DHT_PORT->BSRR = (1U<<DHT_PIN);        // ~30us
  HAL_DelayUs(30);
  dht_pin_in_pu();

  /* Response */
  if (wait_level(0, 200)) return -10;
  if (wait_level(1, 200)) return -11;
  if (wait_level(0, 200)) return -12;

  for (int i=0;i<40;i++){
    if (wait_level(1, 120)) return -13;
    uint32_t t0 = DWT->CYCCNT;
    while (((DHT_PORT->IDR >> DHT_PIN) & 1u) && ((DWT->CYCCNT - t0) < CYCLES_US(120)));
    uint32_t dt_us = (DWT->CYCCNT - t0) / CYCLES_PER_US;
    uint8_t bit = (dt_us > 50) ? 1u : 0u;     // ~26-28us -> 0; ~70us -> 1
    d[i>>3] = (uint8_t)((d[i>>3] << 1) | bit);
    if (((DHT_PORT->IDR >> DHT_PIN) & 1u)){
      if (wait_level(0, 120)) return -14;
    }
  }

  uint8_t sum = (uint8_t)(d[0]+d[1]+d[2]+d[3]);
  if (sum != d[4]) return -15;

  *hum  = d[0];
  *temp = d[2];
  return 0;
}

/* ===== Rain + LED PC13 + SERVO (TIM3 PWM 50Hz) ===== */
static inline void LED_ON(void)  { LED_PORT->BSRR = (1u << (LED_PIN + 16u)); } // 0 -> ON
static inline void LED_OFF(void) { LED_PORT->BSRR = (1u << LED_PIN); }         // 1 -> OFF

static void gpio_init_rain_led_servo(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

  /* PA0: input pull-up */
  RAIN_PORT->CRL &= ~(0xFu << (RAIN_PIN * 4u));
  RAIN_PORT->CRL |=  (0x8u << (RAIN_PIN * 4u));
  RAIN_PORT->ODR |=  (1u << RAIN_PIN);

  /* PC13: output push-pull 2MHz */
  uint32_t sh = (LED_PIN - 8u) * 4u;
  LED_PORT->CRH &= ~(0xFu << sh);
  LED_PORT->CRH |=  (0x2u << sh);
  LED_OFF();

  /* PA6: AF push-pull 50MHz (TIM3_CH1) */
  SERVO_PORT->CRL &= ~(0xFu << (SERVO_PIN * 4u));
  SERVO_PORT->CRL |=  (0xBu << (SERVO_PIN * 4u));
}

static void tim3_pwm_init(void){
  AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP;   // PA6
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  TIM3->PSC  = 7;             // 8MHz/(7+1)=1MHz
  TIM3->ARR  = 20000 - 1;     // 20ms

  TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE);
  TIM3->CCMR1 |=  (6u << 4) | TIM_CCMR1_OC1PE; // PWM1 + preload
  TIM3->CCR1   = 1500;                          // neutral
  TIM3->CCER  |= TIM_CCER_CC1E;
  TIM3->CR1   |= TIM_CR1_ARPE;
  TIM3->EGR    = TIM_EGR_UG;
  TIM3->CR1   |= TIM_CR1_CEN;
}
static void servo_set_us(uint16_t us){
  if(us < 600u)  us = 600u;
  if(us > 2400u) us = 2400u;
  TIM3->CCR1 = us;
  TIM3->EGR  = TIM_EGR_UG;
}
static bool rain_is_active(void){
  uint32_t raw = (RAIN_PORT->IDR & (1u << RAIN_PIN)) ? 1u : 0u;
#if RAIN_ACTIVE_LOW
  return (raw == 0u);  // 0 = mưa
#else
  return (raw == 1u);  // 1 = mưa
#endif
}

/* ===== System init ===== */
static void system_init(void){
  SystemCoreClockUpdate();

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

  DWT_Init();

  /* Peripherals */
  buzzer_init();
  fan_init();
  flame_init_input_pullup();
  dht_pin_in_pu();
  i2c1_init();
  adc1_init();
  usart1_init();

  /* Rain / LED / Servo */
  gpio_init_rain_led_servo();
  tim3_pwm_init();

  /* LCD hello */
  lcd_init();
  lcd_clear();
  lcd_set_cursor(0,0); lcd_print_pad(" Env Monitor ");
  lcd_set_cursor(0,1); lcd_print_pad(" Init... ");
}

/* ===== Main ===== */
int main(void){
  system_init();

  /* Servo quick test: 1ms -> 2ms -> 1.5ms */
  servo_set_us(1000); delay_ms_dwt(800);
  servo_set_us(2000); delay_ms_dwt(800);
  servo_set_us(1500); delay_ms_dwt(400);

  /* Debounce rain: N samples mỗi 10ms (~40ms) */
  const uint8_t N = 4;
  uint8_t rain_cnt = 0, dry_cnt = 0;
  bool    raining  = false;

  /* Initial state */
  if(rain_is_active()){
    raining = true;  LED_ON();  servo_set_us(2000);
  }else{
    raining = false; LED_OFF(); servo_set_us(1000);
  }

  uint16_t tick10ms = 0;   // 0..99 (1s)

  while(1){
    /* ===== Rain debounce mỗi 10 ms ===== */
    bool act = rain_is_active();
    if(act){
      if(rain_cnt < N) rain_cnt++;
      dry_cnt = 0;
      if(!raining && rain_cnt >= N){
        raining = true;
        LED_ON();
        servo_set_us(2000);
      }
    }else{
      if(dry_cnt < N) dry_cnt++;
      rain_cnt = 0;
      if(raining && dry_cnt >= N){
        raining = false;
        LED_OFF();
        servo_set_us(1000);
      }
    }

    /* ===== 1 Hz: đọc cảm biến, alarm, LCD, JSON ===== */
    if(tick10ms == 0){
      uint8_t flame = flame_read();                 // 0 = fire
      uint8_t gas_pct = readGasPercent();
      uint32_t gas_mv = readGasMilliVolt();

      uint8_t tC=0, h=0;
      int dht_ok = dht11_read(&h,&tC);

      /* Alarm logic */
      uint8_t alarm = 0;
      if(dht_ok==0 && tC >= TH_TEMP_C) alarm = 1;
    #if GAS_COMPARE_BY_mV
      if(gas_mv >= TH_GAS_mV)          alarm = 1;
    #else
      if(gas_pct >= TH_GAS_PCT)        alarm = 1;
    #endif
      if(flame == 0)                    alarm = 1;

      if(alarm){ buzz_on(); fan_on(); } else { buzz_off(); fan_off(); }

      /* LCD */
      if(dht_ok==0){
        char ln1[17]; snprintf(ln1, sizeof(ln1), "T:%2uC H:%2u%%", tC, h);
        lcd_set_cursor(0,0); lcd_print_pad(ln1);
      }else{
        char ln1[17]; snprintf(ln1, sizeof(ln1), "DHT ERR:%d", dht_ok);
        lcd_set_cursor(0,0); lcd_print_pad(ln1);
      }
      char ln2[17];
      snprintf(ln2, sizeof(ln2),
              #if GAS_COMPARE_BY_mV
               "GAS:%4lumV",
              #else
               "GAS:%3u%%",
              #endif
               #if GAS_COMPARE_BY_mV
               (unsigned long)gas_mv
               #else
               (unsigned)gas_pct
               #endif
      );
      lcd_set_cursor(0,1); lcd_print_pad(ln2);

      /* UART JSON (không gửi rain) */
      char json[128];
      static uint8_t last_alarm = 0xFF;
      if(dht_ok==0){
        snprintf(json, sizeof(json),
          "{\"t\":%u,\"h\":%u,\"gas\":%u,\"flame\":%u,\"alarm\":%u}\n",
          (unsigned)tC, (unsigned)h, (unsigned)gas_pct, (unsigned)flame, (unsigned)alarm);
      }else{
        snprintf(json, sizeof(json),
          "{\"t\":-1,\"h\":-1,\"gas\":%u,\"flame\":%u,\"alarm\":%u}\n",
          (unsigned)gas_pct, (unsigned)flame, (unsigned)alarm);
      }
      if (alarm != last_alarm) {
        uart1_write(json);
        last_alarm = alarm;
      }
      uart1_write(json);
    }

    delay_ms_dwt(10);
    tick10ms = (uint16_t)((tick10ms + 1) % 100);
  }
}

