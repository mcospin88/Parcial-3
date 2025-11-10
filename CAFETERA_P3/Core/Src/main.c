/*Proyecto: Cafetera - STM32L053R8 (Nucleo) */

#include "stm32l0xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ===================== Slider: Tamaño S/M/L ===================== */
#define SLIDER_ENABLE        1       // 1=usa PA0
#define SLIDER_ADC_CH        0       // ADC_IN0 = PA0
#define SLIDER_SIZE_MIN_PCT  0       // 0 %
#define SLIDER_SIZE_MAX_PCT  100     // 100 %
#define SLIDER_SAMPLE_MS     25      // cada 25 ms
#define SLIDER_EMA_SHIFT     3       // EMA alpha=1/8

/* ===================== Configurables ===================== */
#define LCD_I2C_ADDR_7B      0x27
#define SCROLL_PERIOD_MS     600
#define BUZZ_TONE_HZ         2000

/* Duraciones base bebida (ms) */
#define T_CAPUCHINO_MS       60000
#define T_EXPRESSO_MS        30000
#define T_LATE_MS            80000
#define T_AMERICANO_MS       45000

/* Factores por tamaño (porcentajes) */
#define SZ_PCT_P             80
#define SZ_PCT_M             100
#define SZ_PCT_G             130

/* Botón (PC13) y tiempos */
#define BTN_PORT             GPIOC
#define BTN_PIN              (1u<<13)
#define BTN_DEBOUNCE_MS      5
#define BTN_DOUBLE_MS        500
#define BTN_LONG_MS          1200

/* ===================== Pines ===================== */
/* LEDs: PC0 rojo, PC1 amarillo, PC2 azul */
#define LED_PORT    GPIOC
#define LED_R_PIN   (1u<<0)
#define LED_Y_PIN   (1u<<1)
#define LED_B_PIN   (1u<<2)

/* Buzzer: PA1 TIM21_CH2 AF6 */
#define BUZZ_PORT   GPIOA
#define BUZZ_PIN    (1u<<1)

/* ===== Servo 9g: PB5 = TIM22_CH2 (AF5) ===== */
#define SERVO_PORT        GPIOB
#define SERVO_PIN         (1u<<5)
#define SERVO_AF          5u

/* PWM 50 Hz (20 ms). Con PCLK=16 MHz, PSC=15 => 1 MHz, ARR=20000-1 => 20 ms */
#define SERVO_TIM_PSC     (16-1)
#define SERVO_TIM_ARR     (20000-1)

/* Pulso en microsegundos para servo de rotación continua (FS90R/SG90 mod) */
#define SERVO_US_MIN      1000    // ~CCW
#define SERVO_US_STOP     1500    // stop
#define SERVO_US_MAX      2000    // ~CW

/* Velocidad deseada durante la preparación (ajusta si quieres invertir) */
#define SERVO_RUN_US      SERVO_US_MAX

/* ===================== PCF8574 (mapeo) ===================== */
#define LCD_RS      0x01   // P0
#define LCD_RW      0x02   // P1
#define LCD_EN      0x04   // P2
#define LCD_BL      0x08   // P3

/* ===================== Estados ===================== */
typedef enum { APP_IDLE=0, APP_MENU, APP_SIZE, APP_RUN, APP_PAUSE, APP_DONE } app_state_t;
typedef enum { PH1=0, PH2, PH3 } phase_t;

/* ===================== Globals ===================== */
static volatile uint32_t ms = 0;

static volatile app_state_t app = APP_IDLE;
static app_state_t prev_app = 0xFF;
static volatile phase_t phase = PH1;

static volatile uint8_t  bebida = 0;    // 1..4
static volatile uint8_t  size_sel = 2;  // 1=S, 2=M, 3=L
static volatile bool     bebida_ok = false;
static volatile bool     size_ok   = false;

static volatile uint32_t t_total_ms = 0;
static volatile uint32_t t_phase_ms = 0;
static volatile uint32_t t_rem_ms   = 0;
static volatile uint32_t t_phase_rem_ms = 0;

static volatile uint32_t scroll_ms_acc = 0;
static volatile uint8_t  scroll_idx = 0;

/* Buzzer (2 beeps) */
static volatile uint8_t  buzz_beeps_left = 0;
static volatile uint32_t buzz_phase_ms   = 0;
static volatile bool     buzz_on         = false;

/* UI */
static volatile bool ui_dirty = true;

/* Textos UI */
static const char *idle_text = "Selecciona una opcion   ";
static const char *menu_text = "1=Cap 2=Exp 3=Lat 4=Amr   "; // abreviadas

/* Botón B1 (PC13) */
typedef enum { BTN_EVT_NONE=0, BTN_EVT_SINGLE, BTN_EVT_DOUBLE, BTN_EVT_LONG } btn_evt_t;
static uint8_t  btn_last_raw = 0;
static uint8_t  btn_stable   = 0;
static uint8_t  btn_db_cnt   = 0;
static uint8_t  btn_down     = 0;
static uint32_t btn_down_ms  = 0;
static uint32_t btn_last_rel = 0;

/* ====== Slider (ADC) ====== */
#if (SLIDER_ENABLE)
static volatile uint32_t slider_ema = 0;     // EMA en escala 0..4095 << SLIDER_EMA_SHIFT
static volatile uint16_t slider_raw = 0;     // última lectura cruda 0..4095
static volatile uint16_t slider_val = 0;     // EMA 0..4095 ya escalado
static volatile uint32_t slider_ms_acc = 0;  // acumulador de muestreo
static volatile uint16_t slider_pct = 50;    // % mapeado 0..100 (solo info)
#endif

/* ===================== Prototipos ===================== */
static void clock_init(void);
static void gpio_init(void);
static void i2c1_init_100k(void);
static void lcd_init(void);
static void lcd_cmd(uint8_t c);
static void lcd_data(uint8_t d);
static void lcd_write_nibble(uint8_t en_rs_bl, uint8_t nib);
static void lcd_goto(uint8_t row, uint8_t col);
static void lcd_puts(const char *s);
static void lcd_puts_len(const char* s, uint8_t len);
static void lcd_clear(void);

static void buzz_timer_init(void);
static void buzz_start(void);
static void buzz_stop(void);
static void buzz_schedule_2(void);

static void set_leds(bool r, bool y, bool b);
static void start_bebida(uint8_t b);
static void apply_phase_outputs(void);
static void ui_refresh(void);
static void ui_show_time_remaining(void);

/* Botón */
static void btn_poll_and_dispatch(void);
static void btn_handle_event(btn_evt_t e);

/* Slider / ADC */
#if (SLIDER_ENABLE)
static void adc_init_pa0(void);
static uint16_t adc_read_once(void);
static void slider_sample_1x(void);
static uint16_t map_u16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
#endif

/* Servo */
static void servo_init_pb5(void);
static void servo_write_us(uint16_t usec);
static void servo_start_run(void);
static void servo_stop(void);

/* ===================== Main ===================== */
int main(void)
{
  clock_init();
  gpio_init();
  i2c1_init_100k();
  lcd_init();
  buzz_timer_init();
  servo_init_pb5();
#if (SLIDER_ENABLE)
  adc_init_pa0();
#endif

  /* SysTick 1ms @16MHz */
  SysTick->LOAD = 16000 - 1;
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

  /* Pantalla inicial */
  lcd_clear();
  lcd_goto(0,0); lcd_puts("Selecciona una");
  lcd_goto(1,0); lcd_puts("opcion");
  set_leds(false,false,false);

  while(1) { }   // la lógica va en SysTick
}

/* ===================== Interrupciones ===================== */
void SysTick_Handler(void)
{
  ms++;

  /* Botón B1: polling + eventos */
  btn_poll_and_dispatch();

#if (SLIDER_ENABLE)
  /* Muestreo del slider, % y mapeo S/M/L en APP_SIZE */
  slider_ms_acc++;
  if (slider_ms_acc >= SLIDER_SAMPLE_MS) {
    slider_ms_acc = 0;
    slider_sample_1x();
    slider_pct = map_u16(slider_val, 0, 4095, SLIDER_SIZE_MIN_PCT, SLIDER_SIZE_MAX_PCT);

    if (app == APP_SIZE) {
      if (slider_val < 1365)       size_sel = 1; // S
      else if (slider_val < 2730)  size_sel = 2; // M
      else                         size_sel = 3; // L
      ui_dirty = true;
    }
  }
#endif

  /* Scroll de textos en IDLE/MENU */
  scroll_ms_acc += 1;
  if (scroll_ms_acc >= SCROLL_PERIOD_MS) {
    scroll_ms_acc = 0;
    if (app==APP_IDLE || app==APP_MENU){
      ui_dirty=true;
      scroll_idx++;
    }
  }

  /* Temporizador de proceso */
  if (app == APP_RUN) {
    if (t_rem_ms > 0) {
      t_rem_ms--;
      if (t_phase_rem_ms > 0) {
        t_phase_rem_ms--;
      } else {
        if (phase == PH1) { phase = PH2; t_phase_rem_ms = t_phase_ms; apply_phase_outputs(); }
        else if (phase == PH2) { phase = PH3; t_phase_rem_ms = t_phase_ms; apply_phase_outputs(); }
      }
      if ((t_rem_ms % 200) == 0) ui_dirty = true;  // refresco barra
    } else {
      app = APP_DONE;
      set_leds(false,false,false);
      lcd_clear(); lcd_goto(0,0); lcd_puts("Disfrutalo");
      buzz_schedule_2();
      servo_stop();  // detener al terminar
    }
  }

  /* Buzzer patrón (2 beeps) */
  if (buzz_beeps_left > 0) {
    buzz_phase_ms++;
    if (buzz_on) {
      if (buzz_phase_ms >= 200) { buzz_stop(); buzz_on=false; buzz_phase_ms=0; }
    } else {
      if (buzz_phase_ms >= 200) { buzz_phase_ms=0; buzz_start(); buzz_on=true; buzz_beeps_left--; }
    }
  }

  /* UI */
  if (ui_dirty) { ui_dirty=false; ui_refresh(); }
}

/* ===================== Init / HW ===================== */
static void clock_init(void)
{
  RCC->CR |= RCC_CR_HSION; while(!(RCC->CR & RCC_CR_HSIRDY));
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
}

static void gpio_init(void)
{
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;

  /* LEDs PC0..PC2 output */
  LED_PORT->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2)));
  LED_PORT->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2)));
  set_leds(false,false,false);

  /* Buzzer PA1 AF6 (TIM21_CH2) */
  BUZZ_PORT->MODER &= ~(3u<<(1*2));
  BUZZ_PORT->MODER |=  (2u<<(1*2));
  GPIOA->AFR[0] &= ~(0xF<<(1*4));
  GPIOA->AFR[0] |=  (6u<<(1*4));

#if (SLIDER_ENABLE)
  /* PA0 analógico (ADC_IN0) */
  GPIOA->MODER |= (3u<<(0*2));     // 11 = analog
  GPIOA->PUPDR &= ~(3u<<(0*2));    // sin pull
#endif

  /* Botón B1 PC13: entrada (pulls externos de la Nucleo) */
  BTN_PORT->MODER &= ~(3u<<(13*2));      // input
  BTN_PORT->PUPDR &= ~(3u<<(13*2));      // sin pull interno
}

static void i2c1_init_100k(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  /* PB6/PB7 AF1 Open-Drain Pull-up */
  GPIOB->MODER &= ~((3u<<(6*2))|(3u<<(7*2)));
  GPIOB->MODER |=  ((2u<<(6*2))|(2u<<(7*2)));
  GPIOB->OTYPER |=  ( (1u<<6) | (1u<<7) );
  GPIOB->OSPEEDR |= ((3u<<(6*2))|(3u<<(7*2)));
  GPIOB->AFR[0] &= ~((0xF<<(6*4))|(0xF<<(7*4)));
  GPIOB->AFR[0] |=  ((1u<<(6*4))|(1u<<(7*4)));

  I2C1->CR1 &= ~I2C_CR1_PE;
  I2C1->TIMINGR = 0x00303D5B; // 100kHz @16MHz
  I2C1->CR1 = I2C_CR1_PE;
}

static void buzz_timer_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
  uint32_t pclk = 16000000;
  uint32_t toggles = BUZZ_TONE_HZ * 2;
  uint32_t arr = (pclk / toggles) - 1;
  TIM21->PSC = 0;
  TIM21->ARR = arr;
  TIM21->CCR2 = (arr+1)/2;
  TIM21->CCMR1 &= ~(0xFF00);
  TIM21->CCMR1 |= (6u<<12); // PWM1 CH2
  TIM21->CCER  &= ~TIM_CCER_CC2E; // apagado al inicio
  TIM21->CR1    = TIM_CR1_CEN;
}

/* ===================== Servo (PB5 / TIM22_CH2 AF5) ===================== */
static void servo_init_pb5(void)
{
  /* Habilitar TIM22 y configurar PB5 en AF5 */
  RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;

  SERVO_PORT->MODER &= ~(3u<<(5*2));
  SERVO_PORT->MODER |=  (2u<<(5*2)); // AF
  SERVO_PORT->OTYPER &= ~(1u<<5);
  SERVO_PORT->OSPEEDR |= (3u<<(5*2));
  SERVO_PORT->PUPDR &= ~(3u<<(5*2));
  GPIOB->AFR[0] &= ~(0xFu<<(5*4));
  GPIOB->AFR[0] |=  ((SERVO_AF & 0xF)<<(5*4));

  /* PWM 50 Hz: PSC para 1 MHz, ARR para 20 ms */
  TIM22->CR1 = 0;
  TIM22->PSC = SERVO_TIM_PSC;
  TIM22->ARR = SERVO_TIM_ARR;

  /* Canal 2 en PWM1 */
  TIM22->CCMR1 &= ~(0xFF00);
  TIM22->CCMR1 |= (6u<<12);           // OC2M = PWM1
  TIM22->CCMR1 |= TIM_CCMR1_OC2PE;    // preload
  TIM22->CCER  &= ~TIM_CCER_CC2P;     // polaridad normal
  TIM22->CCR2  = SERVO_US_STOP;       // detenido

  TIM22->CR1 |= TIM_CR1_ARPE;         // auto-reload preload
  TIM22->EGR |= TIM_EGR_UG;           // update

  TIM22->CCER |= TIM_CCER_CC2E;       // habilitar salida CH2
  TIM22->CR1  |= TIM_CR1_CEN;         // arrancar timer (90°-180°)
}

static void servo_write_us(uint16_t usec)
{
  if (usec < SERVO_US_MIN) usec = SERVO_US_MIN;
  if (usec > SERVO_US_MAX) usec = SERVO_US_MAX;
  TIM22->CCR2 = usec;   // con PSC=16-1 => 1 tick = 1 us
}

static void servo_start_run(void)
{
  /* Arranca giro continuo (ajusta SERVO_RUN_US para invertir sentido/velocidad) */
  servo_write_us(SERVO_RUN_US);
}

static void servo_stop(void)
{
  servo_write_us(SERVO_US_STOP);
}

/* ===================== Helpers ===================== */
static void set_leds(bool r, bool y, bool b)
{
  if (r) LED_PORT->ODR |= LED_R_PIN; else LED_PORT->ODR &= ~LED_R_PIN;
  if (y) LED_PORT->ODR |= LED_Y_PIN; else LED_PORT->ODR &= ~LED_Y_PIN;
  if (b) LED_PORT->ODR |= LED_B_PIN; else LED_PORT->ODR &= ~LED_B_PIN;
}

static void buzz_start(void) { TIM21->CCER |= TIM_CCER_CC2E; }
static void buzz_stop(void)  { TIM21->CCER &= ~TIM_CCER_CC2E; }
static void buzz_schedule_2(void) { buzz_beeps_left=2; buzz_phase_ms=0; buzz_on=false; }

/* === Inicio de bebida === */
static void start_bebida(uint8_t b)
{
  switch(b) {
    case 1: t_total_ms = T_CAPUCHINO_MS; break;
    case 2: t_total_ms = T_EXPRESSO_MS;  break;
    case 3: t_total_ms = T_LATE_MS;      break;
    case 4: t_total_ms = T_AMERICANO_MS; break;
    default: t_total_ms = 0; break;
  }

  /* Tamaño según S/M/L (definido por slider) */
  uint32_t pct = (size_sel==1) ? SZ_PCT_P :
                 (size_sel==3) ? SZ_PCT_G : SZ_PCT_M;

  t_total_ms = (t_total_ms * pct) / 100;

  t_phase_ms = t_total_ms / 3;
  t_rem_ms = t_total_ms;
  t_phase_rem_ms = t_phase_ms;
  phase = PH1;
  apply_phase_outputs();

  /* >>> Arrancar servo al iniciar café <<< */
  servo_start_run();
}

/* Indicadores por fase (LEDs); el servo gira todo el tiempo en APP_RUN */
static void apply_phase_outputs(void)
{
  switch(phase) {
    case PH1: // Iniciando
      set_leds(true,false,false);
      break;
    case PH2: // Preparando
      set_leds(false,true,false);
      break;
    case PH3: // Sirviendo
      set_leds(false,false,true);
      break;
  }
}

static void ui_refresh(void)
{
  bool first_draw_of_state = (app != prev_app);
  if (first_draw_of_state) { lcd_clear(); prev_app = app; }

  if (app == APP_IDLE) {
    char temp[17];
    const char* src = idle_text;
    int L = (int)strlen(src);
    for (int i=0;i<16;i++) temp[i] = src[(scroll_idx + i) % L];
    temp[16]=0;
    lcd_goto(0,0); lcd_puts_len(temp,16);

    lcd_goto(1,0);
    lcd_puts_len("B1-> Menu cafe  ",16);
  }
  else if (app == APP_MENU) {
    char temp[17];
    const char* src = menu_text;
    int L = (int)strlen(src);
    for (int i=0;i<16;i++) temp[i] = src[(scroll_idx + i) % L];
    temp[16]=0;
    lcd_goto(0,0); lcd_puts_len(temp,16);

    lcd_goto(1,0);
    if (bebida>=1 && bebida<=4) {
      static const char* sname[5] = {"","Capuchino","Expresso","Latte","Americano"};
      char buf[17];
      snprintf(buf,sizeof(buf),"%u: %-10s",bebida,sname[bebida]);
      buf[16]=0;
      lcd_puts_len(buf,16);
    } else {
      lcd_puts_len("B1: cambiar tipo",16);
    }
  }
  else if (app == APP_SIZE) {
    if (first_draw_of_state) {
      lcd_goto(0,0); lcd_puts_len("Tamano (S/M/L)  ",16);
    }
    lcd_goto(1,0);
    const char* sz = (size_sel==1)?"S":(size_sel==3)?"L":"M";
    char buf[17];
    snprintf(buf,sizeof(buf),"Actual: %s        ",sz);
    lcd_puts_len(buf,16);
  }
  else if (app == APP_RUN) {
    lcd_goto(0,0);
    lcd_puts_len("Cafe preparandose",16);
    ui_show_time_remaining();
  }
  else if (app == APP_PAUSE) {
    /* opcional */
  }
  else if (app == APP_DONE) {
    /* ya se imprimió "Disfrutalo" y se paró el servo */
  }
}

/* --- Línea 2: barra (11) + espacio + "sss" (4) = 16 --- */
static void ui_show_time_remaining(void)
{
  lcd_goto(1,0);

  if (t_total_ms == 0) {
    lcd_puts_len("                ", 16);
    return;
  }

  uint32_t done = t_total_ms - t_rem_ms;
  uint8_t filled = (uint8_t)((done * 11) / t_total_ms);
  if (filled > 11) filled = 11;

  char buf[17];
  for (uint8_t i=0; i<11; i++) buf[i] = (i < filled) ? '#' : '.';
  buf[11] = ' ';

  uint32_t sec = (t_rem_ms + 999) / 1000;
  if (sec > 999) sec = 999;
  snprintf(&buf[12], 5, "%3lus", (unsigned long)sec);

  buf[16] = 0;
  lcd_puts_len(buf, 16);
}

/* ===================== LCD (I2C + PCF8574) ===================== */
static void i2c1_write_byte(uint8_t addr7, uint8_t data)
{
  I2C1->CR2 = (addr7<<1) | (1u<<16) | I2C_CR2_START | I2C_CR2_AUTOEND;
  while(!(I2C1->ISR & I2C_ISR_TXIS));
  I2C1->TXDR = data;
  while(!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR = I2C_ICR_STOPCF;
}

static void lcd_pulse(uint8_t d) { i2c1_write_byte(LCD_I2C_ADDR_7B, d | LCD_EN | LCD_BL); i2c1_write_byte(LCD_I2C_ADDR_7B, d | LCD_BL); }

static void lcd_write_nibble(uint8_t en_rs_bl, uint8_t nib)
{
  uint8_t d = en_rs_bl | ((nib & 0x0F) << 4);
  lcd_pulse(d);
}

static void lcd_cmd(uint8_t c)
{
  uint8_t rs = 0;
  lcd_write_nibble(rs, (c>>4)&0x0F);
  lcd_write_nibble(rs, c & 0x0F);
}

static void lcd_data(uint8_t d8)
{
  uint8_t rs = LCD_RS;
  lcd_write_nibble(rs, (d8>>4)&0x0F);
  lcd_write_nibble(rs, d8 & 0x0F);
}

static void lcd_init(void)
{
  for (volatile int i=0;i<8000;i++);
  lcd_write_nibble(0, 0x03); for (volatile int i=0;i<3000;i++);
  lcd_write_nibble(0, 0x03); for (volatile int i=0;i<3000;i++);
  lcd_write_nibble(0, 0x03); for (volatile int i=0;i<3000;i++);
  lcd_write_nibble(0, 0x02); // 4-bit

  lcd_cmd(0x28); // 2 líneas, 5x8
  lcd_cmd(0x08); // display off
  lcd_cmd(0x01); // clear
  for (volatile int i=0;i<6000;i++);
  lcd_cmd(0x06); // entry mode
  lcd_cmd(0x0C); // display on, cursor off
}

static void lcd_clear(void) { lcd_cmd(0x01); for(volatile int i=0;i<6000;i++); }
static void lcd_goto(uint8_t row, uint8_t col)
{
  uint8_t addr = (row==0)? (0x00+col) : (0x40+col);
  lcd_cmd(0x80 | addr);
}
static void lcd_puts(const char *s) { while(*s) lcd_data((uint8_t)*s++); }
static void lcd_puts_len(const char* s, uint8_t len)
{
  for(uint8_t i=0;i<len;i++){
    char c = s[i];
    if (!c) { lcd_data(' '); }
    else lcd_data((uint8_t)c);
  }
}

/* ===================== Botón B1 ===================== */
static void btn_handle_event(btn_evt_t e)
{
  if (e == BTN_EVT_NONE) return;

  switch(e) {
    case BTN_EVT_SINGLE:
      if (app == APP_IDLE) {
        app = APP_MENU;
        if (bebida < 1 || bebida > 4) bebida = 1;
        bebida_ok = false;
        ui_dirty = true;
      } else if (app == APP_MENU) {
        if (bebida == 0 || bebida >= 4) bebida = 1;
        else bebida++;
        bebida_ok = false;
        ui_dirty = true;
      } else if (app == APP_SIZE) {
        if (bebida < 1 || bebida > 4) bebida = 1;
        start_bebida(bebida);
        app = APP_RUN;
        ui_dirty = true;
      } else if (app == APP_DONE) {
        app = APP_MENU;
        ui_dirty = true;
      }
      break;

    case BTN_EVT_DOUBLE:
      /* no usado */
      break;

    case BTN_EVT_LONG:
      if (app == APP_MENU) {
        app = APP_SIZE;
        ui_dirty = true;
      } else {
        /* Reset total */
        app = APP_IDLE;
        bebida=0; bebida_ok=false; size_sel=2; size_ok=false;
        t_total_ms = t_phase_ms = t_rem_ms = t_phase_rem_ms = 0;
        set_leds(false,false,false);
        servo_stop();
        ui_dirty = true;
      }
      break;

    default: break;
  }
}

static void btn_poll_and_dispatch(void)
{
  uint8_t raw = (BTN_PORT->IDR & BTN_PIN) ? 1 : 0;

  if (raw != btn_last_raw) {
    btn_last_raw = raw;
    btn_db_cnt = 0;
  } else if (btn_db_cnt < BTN_DEBOUNCE_MS) {
    btn_db_cnt++;
  }

  if (btn_db_cnt == BTN_DEBOUNCE_MS) {
    if (!btn_stable && raw) {
      btn_stable = 1;
      btn_down = 1;
      btn_down_ms = ms;
    } else if (btn_stable && !raw) {
      btn_stable = 0;
      uint32_t dur = ms - btn_down_ms;
      btn_down = 0;

      if (dur >= BTN_LONG_MS) {
        btn_handle_event(BTN_EVT_LONG);
        btn_last_rel = ms;
      } else {
        if ((ms - btn_last_rel) <= BTN_DOUBLE_MS) {
          btn_handle_event(BTN_EVT_DOUBLE);
          btn_last_rel = 0;
        } else {
          btn_handle_event(BTN_EVT_SINGLE);
          btn_last_rel = ms;
        }
      }
    }
  }
}

/* ===================== Slider / ADC (PA0) ===================== */
#if (SLIDER_ENABLE)
static void adc_init_pa0(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

  if (ADC1->CR & ADC_CR_ADEN) {
    ADC1->CR |= ADC_CR_ADDIS;
    while (ADC1->CR & ADC_CR_ADEN);
  }

  ADC1->CR |= ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL);

  ADC1->SMPR = 0x07;  // 160.5 cycles -> lectura estable
  ADC1->CHSELR = (1u << SLIDER_ADC_CH);

  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

static uint16_t adc_read_once(void)
{
  ADC1->CR |= ADC_CR_ADSTART;
  while (!(ADC1->ISR & ADC_ISR_EOC));
  return (uint16_t)ADC1->DR;   // 12-bit 0..4095
}

static void slider_sample_1x(void)
{
  slider_raw = adc_read_once();

  if (slider_ema == 0) {
    slider_ema = ((uint32_t)slider_raw) << SLIDER_EMA_SHIFT;
  } else {
    uint32_t ema = slider_ema;
    ema += (((uint32_t)slider_raw << SLIDER_EMA_SHIFT) - ema) >> SLIDER_EMA_SHIFT;
    slider_ema = ema;
  }
  slider_val = (uint16_t)(slider_ema >> SLIDER_EMA_SHIFT);
}

static uint16_t map_u16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  uint32_t num = (uint32_t)(x - in_min) * (uint32_t)(out_max - out_min);
  uint32_t den = (uint32_t)(in_max - in_min);
  return (uint16_t)(out_min + num / den);
}
#endif /* SLIDER_ENABLE */
