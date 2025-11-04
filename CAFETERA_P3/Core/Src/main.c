/*
  Proyecto: Cafetera - STM32L053R8 (Nucleo)
  Corrección completa con keypad funcionando (dos modos), LCD I2C sin parpadeo, 3 fases, stepper 28BYJ-48/ULN2003, buzzer, pausa y doble D.

  ===== Conexiones importantes =====
  LCD I2C (PCF8574):
    - SCL -> PB6   (I2C1_SCL)
    - SDA -> PB7   (I2C1_SDA)
    - VCC -> 3.3V  (recomendado; PB6/PB7 NO son 5V tolerant si el módulo pone pullups a VCC)
    - GND -> GND

  Stepper 28BYJ-48 + ULN2003:
    - IN1 -> PA4
    - IN2 -> PA5
    - IN3 -> PA6
    - IN4 -> PA7
    - VCC ULN2003 -> +5V
    - GND ULN2003 -> GND (común con Nucleo)

  Keypad 4x4:
    MODO POR DEFECTO (KP_INVERT = 0):
      - Filas R0..R3 -> PB0, PB1, PB2, PB3 (salidas)
      - Columnas C0..C3 -> PB4, PB5, PB8, PB9 (entradas con pull-up)
    MODO INVERTIDO (KP_INVERT = 1) [si tu conector viene como C1..C4,R1..R4]:
      - Filas R0..R3 -> PB4, PB5, PB8, PB9 (salidas)
      - Columnas C0..C3 -> PB0, PB1, PB2, PB3 (entradas con pull-up)
*/

#include "stm32l0xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ===================== Configurables ===================== */
#define KP_INVERT            0        // 0: R=PB0..PB3, C=PB4/PB5/PB8/PB9 ; 1: invertido
#define LCD_I2C_ADDR_7B      0x27     // cambia a 0x3F si tu backpack lo usa
#define SCROLL_PERIOD_MS     600      // velocidad de scroll legible
#define KEYPAD_SCAN_MS       5
#define DOUBLE_D_WINDOW_MS   300
#define BUZZ_TONE_HZ         2000

/* Duraciones bebida (ms) */
#define T_CAPUCHINO_MS       60000
#define T_EXPRESSO_MS        30000
#define T_LATE_MS            80000
#define T_AMERICANO_MS       45000

/* ===================== Pines ===================== */
/* LCD I2C1: PB6=SCL, PB7=SDA (AF1) */

/* Keypad: define pines según modo */
#define KP_R_PORT   GPIOB
#define KP_C_PORT   GPIOB

#if (KP_INVERT == 0)
/* Filas: PB0..PB3  |  Columnas: PB4,PB5,PB8,PB9 */
#define KP_R0_PIN   (1u<<0)
#define KP_R1_PIN   (1u<<1)
#define KP_R2_PIN   (1u<<2)
#define KP_R3_PIN   (1u<<3)
#define KP_C0_PIN   (1u<<4)
#define KP_C1_PIN   (1u<<5)
#define KP_C2_PIN   (1u<<8)
#define KP_C3_PIN   (1u<<9)
#else
/* Filas: PB4,PB5,PB8,PB9  |  Columnas: PB0..PB3 */
#define KP_R0_PIN   (1u<<4)
#define KP_R1_PIN   (1u<<5)
#define KP_R2_PIN   (1u<<8)
#define KP_R3_PIN   (1u<<9)
#define KP_C0_PIN   (1u<<0)
#define KP_C1_PIN   (1u<<1)
#define KP_C2_PIN   (1u<<2)
#define KP_C3_PIN   (1u<<3)
#endif

/* LEDs: PC0 rojo, PC1 amarillo, PC2 azul */
#define LED_PORT    GPIOC
#define LED_R_PIN   (1u<<0)
#define LED_Y_PIN   (1u<<1)
#define LED_B_PIN   (1u<<2)

/* Stepper (ULN2003) IN1..IN4 = PA4..PA7 */
#define STP_PORT    GPIOA
#define STP_IN1     (1u<<4)
#define STP_IN2     (1u<<5)
#define STP_IN3     (1u<<6)
#define STP_IN4     (1u<<7)

/* Buzzer: PA1 TIM21_CH2 */
#define BUZZ_PORT   GPIOA
#define BUZZ_PIN    (1u<<1)

/* ===================== PCF8574 (mapeo común) =====================
   P4..P7 -> D4..D7 ; P2=EN ; P1=RW ; P0=RS ; P3=BL
================================================================== */
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
static app_state_t prev_app = 0xFF;     // para evitar lcd_clear frecuente
static volatile phase_t phase = PH1;

static volatile uint8_t  bebida = 0;    // 1..4
static volatile uint8_t  size_sel = 2;  // 1=P,2=M,3=G (default)
static volatile bool     bebida_ok = false;
static volatile bool     size_ok   = false;

static volatile uint32_t t_total_ms = 0;
static volatile uint32_t t_phase_ms = 0;
static volatile uint32_t t_rem_ms   = 0;
static volatile uint32_t t_phase_rem_ms = 0;

static volatile uint32_t scroll_ms_acc = 0;
static volatile uint8_t  scroll_idx = 0;

/* Doble D */
static volatile uint32_t last_D_time = 0;
static volatile bool     d_pressed_recent = false;

/* Keypad */
static volatile char key_event = 0;   // último key detectado (consumido por SysTick)
static uint8_t kp_row = 0;            // fila activa (0..3)
static char kp_map[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
static volatile char kp_last = 0;
static volatile uint8_t kp_stable_cnt = 0;

/* Buzzer (2 beeps) */
static volatile uint8_t  buzz_beeps_left = 0;
static volatile uint32_t buzz_phase_ms   = 0;
static volatile bool     buzz_on         = false;

/* UI flags */
static volatile bool ui_dirty = true;

/* Textos UI */
static const char *idle_text = "Selecciona una opcion   ";
static const char *menu_text = "1=Capuchino 2=Expresso 3=Late 4=Americano   ";

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

static void tim22_init_keypad_scan(void);
static void keypad_drive_row(uint8_t r);
static uint8_t keypad_read_cols(void);

static void buzz_timer_init(void);
static void buzz_start(void);
static void buzz_stop(void);
static void buzz_schedule_2(void);

static void set_leds(bool r, bool y, bool b);
static void start_bebida(uint8_t b);
static void apply_phase_outputs(void);
static void ui_refresh(void);
static void ui_show_time_remaining(void);

static void stepper_apply_phase(uint8_t idx);
static void stepper_set_speed_ms(uint16_t step_period_ms);

/* Stepper control */
static volatile uint8_t  stp_phase = 0;          // 0..7 (half-step)
static volatile uint16_t stp_step_period_ms = 3; // media=3ms, rápida=2ms (ajustable)
static volatile uint16_t stp_acc = 0;

/* ===================== Main ===================== */
int main(void)
{
  clock_init();
  gpio_init();
  i2c1_init_100k();
  lcd_init();
  buzz_timer_init();
  tim22_init_keypad_scan();

  /* SysTick 1ms @16MHz */
  SysTick->LOAD = 16000 - 1;
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

  /* Pantalla inicial */
  lcd_clear();
  lcd_goto(0,0); lcd_puts("Selecciona una");
  lcd_goto(1,0); lcd_puts("opcion");
  set_leds(false,false,false);

  while(1) { __WFI(); }
}

/* ===================== Interrupciones ===================== */
void SysTick_Handler(void)
{
  ms++;

  /* Procesa tecla si hay */
  if (key_event != 0) {
    char k = key_event; key_event = 0;

    if (k >= '1' && k <= '4') {
      if (app == APP_MENU) { bebida = (uint8_t)(k - '0'); bebida_ok = false; ui_dirty = true; }
      if (app == APP_SIZE) { if (k>='1' && k<='3') size_sel = (uint8_t)(k - '0'); ui_dirty = true; }
    }
    else if (k == 'A') { app = APP_MENU; bebida_ok=false; ui_dirty=true; }
    else if (k == 'B') { app = APP_SIZE; ui_dirty=true; }
    else if (k == 'C') {
      if (app == APP_MENU && (bebida>=1 && bebida<=4)) { bebida_ok = true; ui_dirty=true; }
      else if (app == APP_SIZE) { size_ok = true; app = APP_MENU; ui_dirty=true; }
    }
    else if (k == 'D') {
      uint32_t now = ms;
      if (d_pressed_recent && (now - last_D_time) <= DOUBLE_D_WINDOW_MS) {
        app = APP_IDLE;
        bebida=0; bebida_ok=false; size_sel=2; size_ok=false;
        t_total_ms = t_phase_ms = t_rem_ms = t_phase_rem_ms = 0;
        set_leds(false,false,false);
        ui_dirty = true;
        d_pressed_recent = false;
      } else {
        bebida=0; bebida_ok=false; size_ok=false;
        d_pressed_recent = true; last_D_time = now; ui_dirty=true;
      }
    }
    else if (k == '*') {
      if (app == APP_MENU && bebida_ok) { start_bebida(bebida); app = APP_RUN; ui_dirty=true; }
    }
    else if (k == '#') {
      if (app == APP_RUN) {
        app = APP_PAUSE;
        // frenar motor (no energizar bobinas)
        stepper_apply_phase(0xFF); // apaga todo
        lcd_clear(); lcd_goto(0,0); lcd_puts("Pausa");
      } else if (app == APP_PAUSE) {
        app = APP_RUN;
        apply_phase_outputs();
        ui_dirty = true;
      }
    }
  }

  /* Caducidad doble D */
  if (d_pressed_recent && (ms - last_D_time) > DOUBLE_D_WINDOW_MS) d_pressed_recent = false;

  /* Scroll y UI */
  scroll_ms_acc += 1;
  if (scroll_ms_acc >= SCROLL_PERIOD_MS) { scroll_ms_acc = 0; if (app==APP_IDLE || app==APP_MENU){ ui_dirty=true; scroll_idx++; } }

  /* Temporizador de proceso + stepper */
  if (app == APP_RUN) {
    if (t_rem_ms > 0) {
      t_rem_ms--;
      if (t_phase_rem_ms > 0) t_phase_rem_ms--;
      else {
        if (phase == PH1) { phase = PH2; t_phase_rem_ms = t_phase_ms; apply_phase_outputs(); }
        else if (phase == PH2) { phase = PH3; t_phase_rem_ms = t_phase_ms; apply_phase_outputs(); }
      }
      if ((t_rem_ms % 1000) == 0) ui_dirty = true;

      /* Avanza stepper según periodo programado */
      stp_acc++;
      if (stp_acc >= stp_step_period_ms) {
        stp_acc = 0;
        if (phase == PH2 || phase == PH3) {
          stp_phase = (stp_phase + 1) & 0x07; // 0..7
          stepper_apply_phase(stp_phase);
        }
      }

    } else {
      app = APP_DONE;
      stepper_apply_phase(0xFF); // apaga bobinas
      set_leds(false,false,false);
      lcd_clear(); lcd_goto(0,0); lcd_puts("Disfrutalo");
      buzz_schedule_2();
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

/* Keypad scan cada 5 ms (con pequeño settle time tras seleccionar fila) */
void TIM22_IRQHandler(void)
{
  if (TIM22->SR & TIM_SR_UIF) {
    TIM22->SR = ~TIM_SR_UIF;

    keypad_drive_row(kp_row);

    /* Pequeño tiempo de asentamiento (~5-10us) antes de leer columnas */
    for (volatile int d=0; d<200; d++) __NOP();

    uint8_t cols = keypad_read_cols(); // 1=alto, 0=presionado
    char k = 0;
    for (int c=0;c<4;c++) {
      if (((cols>>c)&1)==0) { k = kp_map[kp_row][c]; break; }
    }

    if (k == kp_last) {
      if (k != 0 && kp_stable_cnt < 3) kp_stable_cnt++;
      if (kp_stable_cnt == 2) key_event = k; // estable ~10ms
    } else {
      kp_last = k;
      kp_stable_cnt = 0;
    }

    kp_row = (kp_row + 1) & 0x3;
  }
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

  /* Stepper PA4..PA7 output */
  STP_PORT->MODER &= ~((3u<<(4*2))|(3u<<(5*2))|(3u<<(6*2))|(3u<<(7*2)));
  STP_PORT->MODER |=  ((1u<<(4*2))|(1u<<(5*2))|(1u<<(6*2))|(1u<<(7*2)));
  stepper_apply_phase(0xFF); // apagado

  /* Buzzer PA1 AF (TIM21_CH2 AF6) */
  BUZZ_PORT->MODER &= ~(3u<<(1*2));
  BUZZ_PORT->MODER |=  (2u<<(1*2));
  GPIOA->AFR[0] &= ~(0xF<<(1*4));
  GPIOA->AFR[0] |=  (6u<<(1*4));

  /* Keypad: configurar según KP_INVERT */
#if (KP_INVERT == 0)
  /* Filas PB0..PB3 salida (inactivas en alto) */
  KP_R_PORT->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2)));
  KP_R_PORT->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2)));
  KP_R_PORT->ODR   |=  (KP_R0_PIN|KP_R1_PIN|KP_R2_PIN|KP_R3_PIN);

  /* Columnas PB4,PB5,PB8,PB9 entrada con pull-up */
  KP_C_PORT->MODER &= ~((3u<<(4*2))|(3u<<(5*2))|(3u<<(8*2))|(3u<<(9*2)));
  KP_C_PORT->PUPDR &= ~((3u<<(4*2))|(3u<<(5*2))|(3u<<(8*2))|(3u<<(9*2)));
  KP_C_PORT->PUPDR |=  ((1u<<(4*2))|(1u<<(5*2))|(1u<<(8*2))|(1u<<(9*2)));
#else
  /* Filas PB4,PB5,PB8,PB9 salida (inactivas en alto) */
  KP_R_PORT->MODER &= ~((3u<<(4*2))|(3u<<(5*2))|(3u<<(8*2))|(3u<<(9*2)));
  KP_R_PORT->MODER |=  ((1u<<(4*2))|(1u<<(5*2))|(1u<<(8*2))|(1u<<(9*2)));
  KP_R_PORT->ODR   |=  (KP_R0_PIN|KP_R1_PIN|KP_R2_PIN|KP_R3_PIN);

  /* Columnas PB0..PB3 entrada con pull-up */
  KP_C_PORT->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2)));
  KP_C_PORT->PUPDR &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2)));
  KP_C_PORT->PUPDR |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2)));
#endif
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
  I2C1->TIMINGR = 0x00303D5B; // 100kHz @16MHz (típico L0)
  I2C1->CR1 = I2C_CR1_PE;
}

static void tim22_init_keypad_scan(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
  TIM22->PSC = 16000 - 1;  // 1 kHz
  TIM22->ARR = KEYPAD_SCAN_MS - 1; // 5 ms
  TIM22->DIER = TIM_DIER_UIE;
  TIM22->CR1  = TIM_CR1_CEN;
  NVIC_EnableIRQ(TIM22_IRQn);
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

/* Stepper: half-step 8 fases */
static void stepper_apply_phase(uint8_t idx)
{
  // idx 0..7, 0xFF = apagado
  uint32_t mask = STP_IN1|STP_IN2|STP_IN3|STP_IN4;
  uint32_t out = 0;

  if (idx == 0xFF) { STP_PORT->ODR &= ~mask; return; }

  switch(idx & 7){
    case 0: out = STP_IN1; break;
    case 1: out = STP_IN1|STP_IN2; break;
    case 2: out = STP_IN2; break;
    case 3: out = STP_IN2|STP_IN3; break;
    case 4: out = STP_IN3; break;
    case 5: out = STP_IN3|STP_IN4; break;
    case 6: out = STP_IN4; break;
    case 7: out = STP_IN4|STP_IN1; break;
  }
  STP_PORT->ODR = (STP_PORT->ODR & ~mask) | out;
}

static void stepper_set_speed_ms(uint16_t step_period_ms)
{
  if (step_period_ms < 1) step_period_ms = 1;
  stp_step_period_ms = step_period_ms;
}

static void start_bebida(uint8_t b)
{
  switch(b) {
    case 1: t_total_ms = T_CAPUCHINO_MS; break;
    case 2: t_total_ms = T_EXPRESSO_MS; break;
    case 3: t_total_ms = T_LATE_MS; break;
    case 4: t_total_ms = T_AMERICANO_MS; break;
    default: t_total_ms = 0; break;
  }
  t_phase_ms = t_total_ms / 3;
  t_rem_ms = t_total_ms;
  t_phase_rem_ms = t_phase_ms;
  phase = PH1;
  apply_phase_outputs();
}

/* Velocidades del stepper por fase */
static void apply_phase_outputs(void)
{
  prev_app = APP_RUN; // evitar clears indeseados
  switch(phase) {
    case PH1: // Iniciando (motor apagado)
      set_leds(true,false,false);
      stepper_apply_phase(0xFF);
      lcd_clear(); lcd_goto(0,0); lcd_puts("Iniciando");
      ui_show_time_remaining();
      break;
    case PH2: // Preparando (media)
      set_leds(false,true,false);
      stepper_set_speed_ms(3); // ~3 ms por paso
      lcd_clear(); lcd_goto(0,0); lcd_puts("Preparando");
      ui_show_time_remaining();
      break;
    case PH3: // sirviendo (rapida)
      set_leds(false,false,true);
      stepper_set_speed_ms(2); // ~2 ms por paso
      lcd_clear(); lcd_goto(0,0); lcd_puts("sirviendo");
      ui_show_time_remaining();
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
    for (int i=0;i<16;i++) {
      temp[i] = src[(scroll_idx + i) % L];
    }
    temp[16]=0;
    lcd_goto(0,0); lcd_puts_len(temp,16);
    lcd_goto(1,0); lcd_puts_len("A=Menu * Inicia  ",16);
  }
  else if (app == APP_MENU) {
    char temp[17];
    const char* src = menu_text;
    int L = (int)strlen(src);
    for (int i=0;i<16;i++) {
      temp[i] = src[(scroll_idx + i) % L];
    }
    temp[16]=0;
    lcd_goto(0,0); lcd_puts_len(temp,16);

    lcd_goto(1,0);
    if (bebida>=1 && bebida<=4) {
      static const char* names[5] = {"","Capuchino","Expresso","Late","Americano"};
      char buf[17];
      const char* sz = (size_sel==1)?"P":(size_sel==3)?"G":"M";
      if (bebida_ok) snprintf(buf,sizeof(buf),"%u=%s C:OK %s",bebida,names[bebida],sz);
      else           snprintf(buf,sizeof(buf),"%u=%s C:Guardar %s",bebida,names[bebida],sz);
      lcd_puts_len(buf,16);
    } else {
      lcd_puts_len("Num+C;B=Tam;*=Go",16);
    }
  }
  else if (app == APP_SIZE) {
    if (first_draw_of_state) {
      lcd_goto(0,0); lcd_puts_len("Tam:1=P 2=M 3=G ",16);
      lcd_goto(1,0); lcd_puts_len("C=Guardar  D=Clr",16);
    }
  }
  else if (app == APP_RUN) {
    ui_show_time_remaining(); // línea 1 por fase y línea 2 aquí
  }
  else if (app == APP_PAUSE) {
    // ya se imprimió "Pausa"
  }
  else if (app == APP_DONE) {
    // ya se imprimió "Disfrutalo"
  }
}

/* --- Imprime exactamente 16 chars (sin warnings) --- */
static void ui_show_time_remaining(void)
{
  lcd_goto(1,0);
  uint32_t sec = (t_rem_ms+999)/1000;
  char buf[17];

  if (sec > 9999) {
    const char* s = "Restante: >9999s";
    for (int i=0;i<16;i++) buf[i] = s[i];
    buf[16] = 0;
  } else {
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "Restante: %4lus", (unsigned long)sec);
    size_t n = strlen(tmp);
    if (n > 16) n = 16;
    for (size_t i=0;i<n; i++) buf[i] = tmp[i];
    for (size_t i=n; i<16; i++) buf[i] = ' ';
    buf[16] = 0;
  }
  lcd_puts_len(buf,16);
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

/* Con mapeo P4..P7 = D4..D7: el nibble va <<4 */
static void lcd_write_nibble(uint8_t en_rs_bl, uint8_t nib)
{
  uint8_t d = en_rs_bl | ((nib & 0x0F) << 4);
  lcd_pulse(d);
}

static void lcd_cmd(uint8_t c)
{
  uint8_t rs = 0; // comando
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
  for (volatile int i=0;i<8000;i++); // ~ms de margen de power-up
  // secuencia inicial 4-bit
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

/* ===================== Keypad low-level ===================== */
static void keypad_drive_row(uint8_t r)
{
  uint32_t all = KP_R0_PIN|KP_R1_PIN|KP_R2_PIN|KP_R3_PIN;
  KP_R_PORT->ODR |= all; // alto (inactivo)
  switch(r) {
    case 0: KP_R_PORT->ODR &= ~KP_R0_PIN; break;
    case 1: KP_R_PORT->ODR &= ~KP_R1_PIN; break;
    case 2: KP_R_PORT->ODR &= ~KP_R2_PIN; break;
    case 3: KP_R_PORT->ODR &= ~KP_R3_PIN; break;
  }
}
static uint8_t keypad_read_cols(void)
{
  uint8_t c0 = (KP_C_PORT->IDR & KP_C0_PIN) ? 1:0;
  uint8_t c1 = (KP_C_PORT->IDR & KP_C1_PIN) ? 1:0;
  uint8_t c2 = (KP_C_PORT->IDR & KP_C2_PIN) ? 1:0;
  uint8_t c3 = (KP_C_PORT->IDR & KP_C3_PIN) ? 1:0;
  return (uint8_t)( (c0<<0)|(c1<<1)|(c2<<2)|(c3<<3) );
}
