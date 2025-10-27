# Simulador de Cafetera con STM32

Este proyecto implementa un **firmware para la placa NUCLEO-L053R8** que simula el funcionamiento básico de una máquina de café.
El desarrollo se realizó con **STM32CubeMX** y **STM32CubeIDE**, utilizando la **biblioteca HAL** de STMicroelectronics.

---

## Hardware Requerido

* **Placa de desarrollo:** NUCLEO-L053R8
* **Microcontrolador (MCU):** STM32L053R8Tx

El proyecto utiliza los siguientes elementos:

* Pulsador de usuario `B1` (botón azul)
* LED de usuario `LD2` (verde)
* Tres LEDs externos
* Un servomotor
* Un buzzer (zumbador)

---

## Configuración de Periféricos y Pines

El firmware inicializa los siguientes periféricos:

* **GPIO:** Control de LEDs y lectura del pulsador
* **USART2:** Comunicación serie asíncrona (115200 baudios, 8N1) para mensajes de estado (logging)
* **SYS:** Depuración mediante *Serial Wire Debug*
* **RCC:** Fuente de reloj basada en el oscilador interno MSI

---

### Asignación de Pines (Pinout)

La configuración de pines se define en `main.h` y se detalla en el archivo de configuración `cafetera.ioc`:

| Pin    | Nombre en Placa | Etiqueta en Código | Función                                                                  |
| :----- | :-------------- | :----------------- | :----------------------------------------------------------------------- |
| `PC13` | B1 (Botón Azul) | `B1_Pin`           | Entrada con interrupción (EXTI) en flanco de bajada                      |
| `PA5`  | LD2 (LED Verde) | `LD2_Pin`          | Salida digital (LED de estado)                                           |
| `PA1`  | —               | `led_1_Pin`        | Salida digital (LED externo 1)                                           |
| `PA4`  | —               | `led_2_Pin`        | Salida digital (LED externo 2)                                           |
| `PA6`  | —               | `led_3_Pin`        | Salida digital (LED externo 3)                                           |
| `PA7`  | —               | —                  | Salida de función alternativa (`TIM22_CH2`) para controlar un servomotor |
| `PA2`  | —               | `USART_TX_Pin`     | USART2 TX (Transmisión serie)                                            |
| `PA3`  | —               | `USART_RX_Pin`     | USART2 RX (Recepción serie)                                              |
| `PA13` | —               | `TMS_Pin`          | SYS_SWDIO (Depuración)                                                   |
| `PA14` | —               | `TCK_Pin`          | SYS_SWCLK (Depuración)                                                   |

---

## Funcionalidad del Sistema

El proyecto sigue una arquitectura **controlada por eventos**:

1. **Inicialización:**
   Al inicio, el programa configura el HAL, el reloj del sistema, los GPIO y el USART2.

2. **Espera:**
   El bucle principal (`while(1)`) está vacío, indicando que el microcontrolador entra en modo de espera hasta que ocurra una interrupción.

3. **Activación por Interrupción:**
   El pin `B1` (`PC13`) está configurado como **interrupción externa** (`GPIO_MODE_IT_FALLING`).
   Al presionar el botón azul, se activa la rutina principal.
---

## Entorno de Desarrollo

* **IDE:** STM32CubeIDE
* **Herramienta de configuración:** STM32CubeMX (`cafetera.ioc`)
* **Biblioteca HAL:** STM32Cube FW_L0 V1.12.3
