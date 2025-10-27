# README — Simulador de Cafetera (STM32 NUCLEO-L053R8)

Este proyecto implementa un **simulador de cafetera** usando una Nucleo-L053R8, un **LCD I²C**, **keypad 4×4**, **servo** (para simular mezcla), **buzzer** y **3 LEDs** de estado (rojo/amarillo/azul). El flujo es: **selección de bebida → confirmación → 3 fases** con temporizaciones, servo moviéndose en la fase 2 y buzzer al finalizar.

---

## 1) Hardware y conexiones

**Placa:** NUCLEO-L053R8
**Tensión lógica:** 3.3 V (cuidado con periféricos a 5 V)

### Pines usados

| Función             | Pin Nucleo                      | Nota                                                  |
| ------------------- | ------------------------------- | ----------------------------------------------------- |
| **Buzzer**          | D13 → PA5                       | Comparte pin con **LD2** onboard (parpadea al sonar). |
| **LED rojo**        | D12 → PA6                       | LED fase 1.                                           |
| **LED amarillo**    | A2 → PA4                        | LED fase 2.                                           |
| **LED azul**        | A1 → PA1                        | LED fase 3.                                           |
| **Servo (PWM)**     | D11 → PA7                       | **TIM22_CH1**, 50 Hz.                                 |
| **LCD I²C SDA**     | D14 → PB9                       | PCF8574 (backpack) típico 0x27 u 0x3F.                |
| **LCD I²C SCL**     | D15 → PB8                       | I²C1 AF4 open-drain + pull-up.                        |
| **Keypad filas**    | D9 PC7, D8 PA9, D7 PA8, D6 PB10 | Entradas con **pull-up**.                             |
| **Keypad columnas** | D5 PB4, D4 PB5, D3 PB3, D2 PA10 | Salidas.                                              |

**Alimentación recomendada:**

* **Placa**: por USB (U5V) o VIN si usas batería/regulador externo.
* **Servo**: **5 V dedicado** (≥1 A) con **GND común** a la Nucleo.
* **LCD I²C**: alimenta a **3.3 V** si su backpack lo permite (mejor para no subir SDA/SCL a 5 V). Si solo trabaja a 5 V, usa **level shifter** o pull-ups a 3.3 V.

---

## 2) Software (entorno)

* **STM32CubeIDE** (o STM32CubeMX + tu toolchain GCC ARM).
* **HAL STM32L0**.

### Habilitar módulos HAL

En `Core/Inc/stm32l0xx_hal_conf.h` asegúrate de tener:

```c
#define HAL_I2C_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
```

Y que tu proyecto incluya los sources de HAL:

```
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c.c
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim.c
// (opcionales) _i2c_ex.c y _tim_ex.c
```

---

## 3) Estructura del código

Todo vive en `main.c`. Los bloques principales:

* **Init de HAL y reloj** (`SystemClock_Config`)
* **Init GPIO**, **I2C1 (PB9/PB8)** y **TIM22 PWM (PA7)**
* **Driver LCD (HD44780 vía PCF8574)** en modo 4-bit por I²C
* **Escaneo Keypad 4×4** (salidas por columnas, entradas con pull-up en filas)
* **Servo** (50 Hz) con helpers para fijar ángulo y “sweep” de mezcla
* **Buzzer/LEDs** helpers
* **Lógica de cafetera**: menú → confirmación → fases

---

## 4) Flujo de ejecución (paso a paso)

1. **Arranque**

   * `HAL_Init()` + `SystemClock_Config()`.
   * `MX_GPIO_Init()`, `MX_I2C1_Init()`, `MX_TIM22_Init()`.
   * `lcd_init()`, `servo_init_50Hz()`, `leds_off()`.
   * Muestra **“Cafetera Ready”**.

2. **Menú principal**

   * LCD muestra: `1 Capuc 2 Latte` en la 1ª línea y `3 Macch 4 Amer` en la 2ª.
   * El sistema espera una tecla **1..4** del keypad.

3. **Confirmación**

   * Se muestra el nombre de la bebida y `#=OK *=Cancel`.
   * Si se presiona `#` en ≤15 s → **confirmado**. Si `*` o timeout → **cancelado** y vuelve al menú.

4. **Fases de preparación**

   * **Fase 1** (LED **rojo** ON) — “precarga/molido”. Duración `PHASE1_MS`.
   * **Fase 2** (LED **amarillo** ON) — “extracción + mezcla”.

     * El **servo** hace un **barrido suave** alrededor de 90° con amplitud `MIX_SWEEP_DEG` y paso cada `MIX_SPEED_MS`, durante `PHASE2_MS`.
   * **Fase 3** (LED **azul** ON) — “finalización”. Duración `PHASE3_MS`.
   * Al terminar: LCD **“Listo!”** + **3 beeps** de buzzer.

5. **Regreso al menú** para nueva selección.

---

## 5) Parámetros configurables

En la cabecera de `main.c`:

```c
#define PHASE1_MS     6000   // ms de Fase 1
#define PHASE2_MS    10000   // ms de Fase 2
#define PHASE3_MS     4000   // ms de Fase 3
#define MIX_SWEEP_DEG   50   // amplitud de mezcla del servo
#define MIX_SPEED_MS    20   // delay entre pasos del servo
#define LCD_ADDR   (0x27<<1) // cambia a (0x3F<<1) si tu backpack es 0x3F
```

> Si el LCD no muestra nada: cambia `LCD_ADDR` a `0x3F<<1`.

---

## 6) Detalles de implementación

### 6.1 LCD I²C (PCF8574)

* Modo **4-bit**: se envían **nibbles** altos y bajos con **EN** y **RS** controlados por el expander.
* Funciones clave:

  * `lcd_init()`, `lcd_clear()`, `lcd_set_cursor(row,col)`, `lcd_print(str)`.

### 6.2 Keypad 4×4

* **Columnas** (D5 PB4, D4 PB5, D3 PB3, D2 PA10) como **salidas**:

  * Técnica de **escaneo**: se pone **1 columna a LOW** y se leen las **filas**.
* **Filas** (D9 PC7, D8 PA9, D7 PA8, D6 PB10) como **entradas con pull-up**:

  * Tecla presionada se lee como **LOW**.
* `keypad_getkey()` devuelve el **char** mapeado (`'0'..'9','A'..'D','*','#'`) o `0` si no hay tecla.

### 6.3 Servo (TIM22 → PA7)

* PWM a **50 Hz** (periodo 20 ms).
* **Pulso**: 1.0 ms ≈ 0°, 1.5 ms ≈ 90°, 2.0 ms ≈ 180°.
* `servo_set_deg(deg)` calcula el **duty** en microsegundos (si el timer corre ~1 MHz, 1 tick ≈ 1 µs).
* `servo_mix_sweep(center, amplitude, ms)` mueve de ida y vuelta para simular mezcla.

### 6.4 Buzzer y LEDs

* GPIO **salida**.
* `buzzer_beep(ms)` pone PA5 HIGH por `ms` y luego LOW.
* LEDs con helpers para encender/apagar por fase.

---

## 7) Cómo compilar y ejecutar

1. **Crear proyecto** en STM32CubeIDE para **STM32L053R8**.
2. **Configurar periféricos** (o usar los `MX_*_Init()` ya provistos):

   * I²C1 en **PB9/PB8** (SDA/SCL, AF4, OD + pull-up).
   * TIM22 CH1 en **PA7** (PWM).
   * GPIO para LEDs/buzzer y keypad como en la tabla.
3. Asegúrate en `stm32l0xx_hal_conf.h` de tener **I2C** y **TIM** habilitados.
4. **Build** y flashea por **ST-LINK** (USB).
5. Al iniciar debe aparecer **“Cafetera Ready”** en el LCD.

---

## 8) Pruebas rápidas (diagnóstico)

* **LCD**: si no muestra nada, prueba **`LCD_ADDR` 0x27/0x3F**, revisa SDA/SCL, **pull-ups a 3.3 V**, y que esté alimentado (GND común).
* **Servo**: al iniciar debería “centrarse” (~90°). Si vibra o reinicia la placa, usa **fuente de 5 V dedicada** y **GND común**.
* **Keypad**: si las teclas no corresponden, es probable que filas/columnas estén permutadas; ajusta los arrays de pines o reubica cables.
* **Buzzer**: en PA5; recuerda que **LD2** (LED verde onboard) parpadea en el mismo pin.

---

## 9) Errores comunes y solución

* **`fatal error: stm32l0xx_hal_tim.h: No such file or directory`**
  → Faltan archivos HAL TIM o include path. Re-genera con CubeMX o añade:

  * `Drivers/STM32L0xx_HAL_Driver/Inc/stm32l0xx_hal_tim.h`
  * `Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim.c`
    y las rutas de include.

* **`unknown type name 'I2C_HandleTypeDef'` / `TIM_* undeclared`**
  → Habilita en `stm32l0xx_hal_conf.h`:
  `#define HAL_I2C_MODULE_ENABLED` y `#define HAL_TIM_MODULE_ENABLED`.

* **ST-LINK no se conecta / LD1 titila**
  → Problema de **alimentación** (servo chupando corriente). Usa fuente 5 V separada, **GND común**, cable USB de **datos**, actualiza firmware del ST-LINK (STM32CubeProgrammer).

---

## 10) Personalización rápida

* **Tiempos de fases**: `PHASE1_MS`, `PHASE2_MS`, `PHASE3_MS`.
* **Movimiento del servo**: `MIX_SWEEP_DEG`, `MIX_SPEED_MS`.
* **Nombres del menú**: edita `bebida_name()` o el menú donde se imprimen las opciones.

---

## 11) Seguridad y buenas prácticas

* **Nunca** alimente el pin **5V** de la Nucleo con **>5 V**. Si usas batería de **7 V**, entra por **VIN** o usa un **regulador a 5 V**.
* **GND común** siempre entre placa, servo y periféricos.
* Si el LCD I²C se alimenta a **5 V**, asegúrate de que **SDA/SCL** no suban a 5 V (usa **shifter** o pull-ups a 3.3 V).