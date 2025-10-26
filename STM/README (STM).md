# MDP Robot — STM32F407 Firmware

## Overview
This repository contains the embedded firmware for the NTU **CE/CZ3004 Multi-Disciplinary Design Project (MDP)** robot.  
It handles motor drive, steering, encoder feedback, IR and ultrasonic sensing, OLED display, and UART telemetry using **STM32F407VET6** and **FreeRTOS**.

The firmware supports:
- DC motor control with PWM and encoder feedback  
- Servo steering (PWM)  
- **Infrared distance sensing** (analog, ADC-based)  
- **Ultrasonic distance sensing** (HC-SR04 style via GPIO interrupt or timer capture)  
- OLED I²C display  
- UART logging and debugging  
- Modular FreeRTOS task-based design

---

## Hardware Overview
- **Controller:** STM32F407VET6 (Wheeltec board)  
- **Drive Motors:** 12 V DC with 1:30 gearbox and hall encoders  
- **Steering Servo:** TD-8120MG (4.8–6.6 V), pulse range 0.5–2.5 ms @ 50 Hz  
- **IR Sensors:** Analog Sharp GP2Y0A21YK or equivalent (ADC inputs)  
- **Ultrasonic Sensor:** HC-SR04 (trigger/echo pins)  
- **OLED:** I²C (on I²C2)  
- **Power:** 12 V battery pack (5 V regulated for logic/servo)

**Peripherals used**
| Peripheral | Function | Channels/Pins | Notes |
|-------------|-----------|---------------|-------|
| TIM2 | Encoder A | CH1/CH2 | Left wheel |
| TIM5 | Encoder D | CH1/CH2 | Right wheel |
| TIM4 | PWM drive A | CH3/CH4 | Motor A H-bridge |
| TIM1 | PWM drive D | CH3/CH4 | Motor D H-bridge |
| TIM3 | Servo PWM | CH4 (PC9) | Steering |
| **ADC1** | IR sensors | CH10 (PC0), CH11 (PC1) etc. | Analog distance sensing |
| **TIM9 / GPIO** | Ultrasonic | e.g. PB8 Trig, PB9 Echo | Timer capture or EXTI |
| I²C2 | OLED | PB10 (SCL), PB11 (SDA) |
| USART2 | Serial (BT/console) | – | 9600 bps |
| USART3 | Serial (Plotter/log) | – | 115200 bps |

---

## Sensor Modules

### **Infrared Sensors (ADC)**
- Each IR sensor outputs an analog voltage inversely proportional to distance.  
- The firmware samples via `adc1_read_once()` and converts it to cm using a power-fit function:

  ```c
  int32_t IrConvert(int32_t raw) {
      // Example power-law fit for Sharp GP2Y0A21
      return (int32_t)(9460 * pow(raw, -1.22));  // cm estimate
  }
  ```

- A **median-9 filter** smooths noise before conversion.  
- Typical range: 10–60 cm.  
- Used for **wall hugging**, **alignment**, and **obstacle detection**.

### **Ultrasonic Sensor**
- Standard HC-SR04-type module with **trigger** and **echo** pins.  
- Distance (cm) = (Echo time × Speed of sound) / 2.  
- Uses a FreeRTOS task or interrupt to measure echo pulse width via **TIM input capture** or **micros()-style timing**.  
- Effective range: 2 cm – 400 cm.  
- Typically used for **front obstacle detection** or **stop condition**.

---

## Repository Structure
| File | Description |
|------|-------------|
| `Core/Src/main.c` | System init, peripheral setup, FreeRTOS task creation |
| `Core/Inc/main.h` | Pin definitions, macros |
| `Core/Src/stm32f4xx_it.c/.h` | Interrupt service routines (ADC, TIM, EXTI) |
| `Core/Src/stm32f4xx_hal_msp.c` | MSP config for timers, UART, I²C, ADC |
| `Core/Src/freertos.c` | Task creation |
| `Core/Inc/FreeRTOSConfig.h` | RTOS config |

---

## FreeRTOS Tasks (Examples)
- **Distance:** reads IR + ultrasonic sensors and prints to UART/OLED  
- **Motors:** PID or direct PWM drive control  
- **Encoder:** reads wheel counts for odometry  
- **ServoMotors:** steering PWM updates  
- **Show:** OLED display refresh  

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

## Building & Flashing
1. Open the project in **STM32CubeIDE**.  
2. Ensure all necessary peripherals (ADC1, TIMx, I²C2) are enabled.  
3. Connect **ST-LINK** via SWD.  
4. **Important:** disconnect the servo’s power line while flashing to avoid current spikes.  
5. Flash the firmware and power the board via the 12 V supply.

---

## Quick Hardware Verification
1. **PWM outputs:** observe PB8/PB9 and PE13/PE14 with an oscilloscope; duty cycle should vary with speed command.  
2. **Encoders:** rotate wheels manually; verify counter increments in TIM2/TIM5 registers.  
3. **Servo:** confirm sweep range 0.5–2.5 ms pulse (~±36° steering).  
4. **IR sensors:** check UART log for cm readings matching object distance.  
5. **Ultrasonic:** verify readings change with object movement (10–50 cm).  
6. **OLED:** check I²C2 communication at 100 kHz with proper pull-ups.  

---

## Safety Notes
- Disconnect servo power when programming via USB.  
- Avoid back-powering through USB—always power via the main 12 V supply first.  
- Ensure grounds are common between motor driver, sensors, and MCU.

---

## Tuning & Control
- Use PID tuning for smoother speed control.  
- Start with a base PWM (≈ 250–300 minimum to overcome deadzone).  
- Clamp max PWM to around 7000 for balanced speed and safety.  
- Log encoder and distance data via UART3 to tune motion and sensor thresholds.

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
| IR values unstable | Noise or power fluctuation | Add 0.1 µF capacitor near sensor |
| Ultrasonic always zero | Echo pin misconfigured | Check EXTI/IC capture setup |
| OLED not displaying | I²C wiring or address issue | Check PB10/PB11, 0x78 address |
| UART noise | Baud mismatch | Verify 9600/115200 settings |

---

## Credits
Developed using **STM32 HAL** and **FreeRTOS (1 kHz tick)** under **STM32CubeIDE**.  
Implements modular control for motors, encoders, steering, IR/ultrasonic sensors, and OLED display for the MDP autonomous robot platform.
