# MDP Robot — STM32F407 Firmware

## Overview
This repository contains the embedded firmware for the NTU **CE/CZ3004 Multi-Disciplinary Design Project (MDP)** robot.  
It handles drive control, steering, encoder feedback, OLED display, and serial communication using **STM32F407VET6** and **FreeRTOS**.

The firmware supports:
- DC motor control with PWM and encoder feedback  
- Servo steering (PWM)  
- OLED I²C display  
- UART logging and debugging  
- Modular FreeRTOS task-based design

---

## Hardware Overview
- **Controller:** STM32F407VET6 (Wheeltec board)  
- **Drive Motors:** 12 V DC with 1:30 gearbox and hall encoders  
- **Steering Servo:** TD-8120MG (4.8–6.6 V), pulse range 0.5–2.5 ms @ 50 Hz  
- **OLED:** I²C (on I²C2)  
- **Power:** 12 V battery pack (5 V regulated for logic/servo)  

**Peripherals used**
| Peripheral | Function | Channels | Notes |
|-------------|-----------|-----------|-------|
| TIM2 | Encoder A | CH1/CH2 | Left wheel |
| TIM5 | Encoder D | CH1/CH2 | Right wheel |
| TIM4 | PWM drive A | CH3/CH4 | Motor A H-bridge |
| TIM1 | PWM drive D | CH3/CH4 | Motor D H-bridge |
| TIM3 | Servo PWM | CH4 | Steering |
| I²C2 | OLED | PB10 (SCL), PB11 (SDA) |
| USART2 | Serial (BT/console) | – | 9600 bps |
| USART3 | Serial (Plotter/log) | – | 115200 bps |

---

## Repository Structure
| File | Description |
|------|-------------|
| `Core/Src/main.c` | System init, peripheral setup, FreeRTOS task creation |
| `Core/Inc/main.h` | Pin definitions, macros |
| `Core/Src/stm32f4xx_it.c/.h` | Interrupt service routines |
| `Core/Src/stm32f4xx_hal_msp.c` | MSP configuration for timers, UART, I²C |
| `Core/Src/freertos.c` | FreeRTOS task creation hooks |
| `Core/Inc/FreeRTOSConfig.h` | RTOS tick and memory configuration |

---

## FreeRTOS Tasks
Typical tasks defined or available:
- **Distance** — distance sensing and display updates  
- **Motors** — PID or direct PWM drive control  
- **Encoder** — read wheel encoders for odometry  
- **ServoMotors** — steering servo control  
- **Show** — OLED display output  

You can enable or disable specific tasks in `main.c` according to your testing setup.

---

## Motion Constants
```c
#define WHEEL_DIAMETER_MM  65.0f
#define WHEEL_CIRC_MM      (3.14159f * WHEEL_DIAMETER_MM)
#define COUNTS_PER_REV     1560
```
- Encoder counts correspond to **11 PPR × 30 gear ratio × 4x decode ≈ 1320 counts/rev**; adjust this constant after calibration.
- Distance and speed are derived from encoder feedback in cm/s or RPM as required.

---

## UART & Display
- **UART3:** high-rate CSV output for serial plotter (e.g., `t_ms,error,duty,rps`).  
- **UART2:** Bluetooth or console logs.  
- **OLED:** I²C display showing distance, direction, or debugging data.

---

## Timer & Pin Map
| Function | Timer | Channel | Pin |
|-----------|--------|----------|------|
| Motor A PWM | TIM4 | CH3/CH4 | PB8 / PB9 |
| Motor D PWM | TIM1 | CH3/CH4 | PE13 / PE14 |
| Encoder A | TIM2 | CH1/CH2 | PA15 / PB3 |
| Encoder D | TIM5 | CH1/CH2 | PA0 / PA1 |
| Servo PWM | TIM3 | CH4 | PC9 |
| OLED I²C | I²C2 | – | PB10 / PB11 |

---

## Building & Flashing
1. Open the project in **STM32CubeIDE**.  
2. Ensure all necessary tasks are created in `MX_FREERTOS_Init()`.  
3. Connect **ST-LINK** via SWD.  
4. **Important:** disconnect the servo’s power line while flashing to avoid current spikes.  
5. Flash the firmware and power the board via the 12 V supply.

---

## Quick Hardware Verification
1. **PWM outputs:** observe PB8/PB9 and PE13/PE14 with an oscilloscope; duty cycle should vary with speed command.  
2. **Encoders:** rotate wheels manually; verify counter increments in TIM2/TIM5 registers.  
3. **Servo:** confirm sweep range 0.5–2.5 ms pulse (~±36° steering).  
4. **OLED:** check I²C2 communication at 100 kHz with proper pull-ups.  

---

## Safety Notes
- Disconnect servo power when programming via USB.  
- Avoid back-powering through USB—always power via the main 12 V supply first.  
- Ensure grounds are common between motor driver and MCU.

---

## Tuning & Control
- Use PID tuning for smoother speed control.  
- Start with a base PWM (≈ 250–300 minimum to overcome deadzone).  
- Clamp max PWM to around 7000 for balanced speed and safety.  
- Log encoder counts via UART3 to tune control loops.

---

## Project Timeline (MDP)
| Week | Deliverable |
|------|--------------|
| 7 | System checklist demo |
| 8 | Task 1 — Autonomous movement & image recognition |
| 9 | Task 2 — Fastest Car |
| 10 | 5-min final presentation video |

---

## Troubleshooting
| Symptom | Likely Cause | Fix |
|----------|---------------|-----|
| Servo twitches during flash | Shared 5 V line surge | Disconnect servo while flashing |
| Wrong distance counts | Encoder constant mismatch | Re-measure encoder counts per rev |
| OLED not displaying | I²C wiring or address issue | Check PB10/PB11, 0x78 address |
| UART noise | Baud mismatch | Verify 9600/115200 settings |

---

## Credits
Developed using **STM32 HAL** and **FreeRTOS (1 kHz tick)** under **STM32CubeIDE**.  
Implements modular control for motors, encoders, steering, and OLED display for the MDP autonomous robot platform.
