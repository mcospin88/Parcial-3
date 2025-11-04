# Proyecto “Cafetera” — STM32L053R8 (Nucleo)

Este proyecto implementa el firmware para una cafetera didáctica basada en un microcontrolador **STM32L053R8** (Nucleo). Utiliza un **keypad 4×4**, una pantalla **LCD 16×2 con interfaz I2C (PCF8574)**, un **motor paso a paso 28BYJ-48 con driver ULN2003**, tres **LEDs de estado** y un **buzzer**.  

Todo el funcionamiento está basado en interrupciones: no se emplean retardos de bloqueo (`delay`) ni bucles de espera activa.

---

## 🎛️ Características principales

- Interfaz de usuario mediante LCD desplazable.
- Menú de selección con soporte para:
  - `A`: acceder al menú de bebidas.
  - `1–4`: seleccionar bebida.
  - `B`: seleccionar tamaño.
  - `C`: confirmar selección.
  - `D`: borrar selección (doble `D` reinicia).
  - `*`: iniciar proceso.
  - `#`: pausar/reanudar.
- Proceso dividido en 3 fases:
  1. **Iniciando** — LED rojo, sin motor.
  2. **Preparando** — LED amarillo, motor a velocidad media.
  3. **Sirviendo** — LED azul, motor rápido.
- Tiempo total según bebida seleccionada:
  | Bebida      | Tiempo total |
  |-------------|--------------|
  | Capuchino   | 60 s         |
  | Expresso    | 30 s         |
  | Late        | 80 s         |
  | Americano   | 45 s         |
- Al terminar: mensaje **“Disfrútalo”** en la LCD y sonido de **2 beeps** en el buzzer.

---

## 🛠️ Hardware necesario

### Placa de desarrollo
- **Nucleo-L053R8** (STM32L053R8T6 @ 16 MHz HSI).

### Conexiones destacadas

#### LCD I2C (con módulo PCF8574)
| Pin Nucleo | Señal | Descripción |
|------------|--------|-------------|
| PB6        | SCL    | I2C1 Clock  |
| PB7        | SDA    | I2C1 Data   |
| 3.3V       | VCC    | Alimentación del módulo |
| GND        | GND    | Tierra común |

> ⚠️ Nota: Si el módulo trae resistencias pull-up a 5V, debe alimentarse a 3.3V para no dañar los pines.

#### Motor 28BYJ-48 + ULN2003
| Pin Nucleo | Señal | ULN2003 |
|------------|-------|---------|
| PA4        | IN1   | Step 1  |
| PA5        | IN2   | Step 2  |
| PA6        | IN3   | Step 3  |
| PA7        | IN4   | Step 4  |

Alimentación del ULN2003: +5V con GND común.

#### Keypad 4×4 (por defecto `KP_INVERT=0`)
| Línea | Puerto | Pines |
|-------|--------|-------|
| Filas (R0–R3) | GPIOB | PB0, PB1, PB2, PB3 |
| Columnas (C0–C3) | GPIOB | PB4, PB5, PB8, PB9 |

> Si el teclado viene invertido (C primero, luego R), definir `#define KP_INVERT 1` en el código.

#### LEDs y buzzer
| Módulo    | Pin Nucleo |
|-----------|------------|
| LED rojo  | PC0        |
| LED amarillo | PC1     |
| LED azul  | PC2        |
| Buzzer (PWM) | PA1 - TIM21 CH2 |

---

## ⚙️ Interrupciones configuradas

| IRQ      | Frecuencia | Función principal |
|----------|------------|------------------|
| **SysTick** | 1 ms | Control de temporizadores, scroll, FSM general, motor, buzzer |
| **TIM22**   | 5 ms | Escaneo del keypad con antirrebote |

> El bucle principal usa `__WFI()` para que el micro entre en reposo y sea despertado por interrupciones.

---

## 🧩 Configuración en STM32CubeIDE

1. Instalar el paquete **STM32Cube FW_L0**  
   (Help → Manage embedded software packages → STM32L0)

2. Añadir rutas de inclusión en:  
   `Project → Properties → C/C++ Build → MCU GCC Compiler → Include paths`

   ```plaintext
   .../Drivers/CMSIS/Include  
   .../Drivers/CMSIS/Device/ST/STM32L0xx/Include
````

3. Definir `STM32L053xx` en:
   `Project → Properties → C/C++ Build → MCU GCC Compiler → Preprocessor`

4. No definir manualmente `SysTick_Handler` en `main.c`.

---

## 🔧 Parámetros personalizables

Ubicados al inicio de `main.c`:

```c
#define KP_INVERT            0        // 1 si las líneas del keypad están invertidas
#define LCD_I2C_ADDR_7B      0x27     // Dirección I2C de la pantalla
#define SCROLL_PERIOD_MS     600      // Velocidad del scroll
#define T_CAPUCHINO_MS       60000
#define T_EXPRESSO_MS        30000
#define T_LATE_MS            80000
#define T_AMERICANO_MS       45000
```

---

## ▶️ Flujo de uso

1. Al encender, aparece “Selecciona una opción”.
2. Presionar `A` para acceder al menú.
3. Presionar `1–4` para seleccionar la bebida y `C` para confirmar.
4. (Opcional) Presionar `B` para seleccionar el tamaño P, M o G, y `C` para guardar.
5. Presionar `*` para iniciar el proceso.
6. Presionar `#` para pausar/reanudar.
7. Presionar `D` para borrar selección actual o `D` dos veces para reiniciar.
8. Al terminar, se muestra "Disfrútalo" y suena el buzzer.

---

## 🪛 Solución de problemas

| Problema                    | Posible causa                             |
| --------------------------- | ----------------------------------------- |
| LCD enciende pero sin texto | Dirección I2C errónea, error en SDA/SCL   |
| Keypad no detecta teclas    | Configuración invertida (`KP_INVERT`)     |
| Motor no gira               | Fallo en conexiones PA4–PA7 o falta de 5V |

---
