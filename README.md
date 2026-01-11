# RTOS-Based Grain Drier

![Platform](https://www.nxp.com/products/LPC1768FBD100)
![RTOS](https://www.keil.com/support/man/docs/rlarm/rlarm_ar_artxarm.htm)
![IDE](https://www.youtube.com/watch?v=PDlRiDz8yB8&pp=ygUaa2VpbCB1dmlzaW9uIDTigIsgZG93bmxvYWQ%3D)
![Language](https://www.open-std.org/jtc1/sc22/wg14/)

An embedded systems project demonstrating the use of a **Real-Time Operating System (RTOS)** in a grain drying application using the **RTL-RTX kernel** on the **LPC1768 (ARM Cortex-M3)** microcontroller.

---

## Overview

This project showcases how RTOS concepts such as multitasking, scheduling, and synchronization can be applied to a real-world grain drying system. The application integrates multiple sensors and motors, implemented in both **bare-metal** and **RTOS-based** environments for comparison and learning.

---

## Hardware Components

- **LPC1768** – ARM Cortex-M3 Microcontroller  
- **DHT11** – Temperature sensor  
- **HW080** – Capacitive moisture sensor  
- **A3144** – Hall-effect sensor  
- **DC Motor** – Airflow control  
- **Stepper Motor** – Mechanical actuation  
- **LCD Display** – User interface  

---

## Software & Tools

- **RTOS**: RTL-RTX Kernel (Keil)
- **IDE**: Keil µVision 4
- **Programming Language**: Embedded C
- **Target MCU**: LPC1768

---

## Required System Files

The following files are mandatory for RTOS operation and LPC1768 system initialization. They are already included in the repository:

 `RTX_Config_CM.c`
 `system_lpc17xx.c`
 `system_lpc17xx.s`

---

## Setup Instructions (Keil µVision 4)

1. Create a **new project** in Keil µVision 4.  
   - It is recommended to create **separate projects** to understand each sensor individually.

2. Add the required source file(s) along with:
   - RTX configuration files
   - LPC1768 system files

3. For LCD functionality:
   - Add `lcd.h`
   - Add `lcd.s`

4. Open **Options for Target**:
   - Enable and select the **RTX Kernel**.

5. Build the project and flash it to the LPC1768 board.

---

## Source File Descriptions

| File Name | Description |
|-----------|-------------|
| `DcMotor.c` | DC motor testing |
| `dht11_OS.c` | DHT11 temperature sensor (RTOS environment) |
| `FanError_main.c` | **Main application file** |
| `hall_sensor_os.c` | Hall-effect sensor (RTOS environment) |
| `hall_sensor.c` | Hall-effect sensor (bare-metal) |
| `lcd_interface_main.c` | LCD with all sensors *(work in progress)* |
| `lcd.c` | LCD driver source file |
| `main_lcd.c` | LCD integration with temperature and moisture sensors |
| `main.c` | Temperature and moisture sensor integration |
| `moistLM393.c` | Moisture sensor (bare-metal) |
| `RTX_Conf_CM.c` | RTX kernel configuration |
| `sensor_dht11.c` | DHT11 sensor (bare-metal) |
| `sensor_hw080_OS.c` | Moisture sensor (RTOS environment) |
| `SensorStepperDc.c` | DC motor + stepper + sensors *(unoptimized)* |
| `stepper.c` | Stepper motor (bare-metal) |
| `StepperAndSenor_main.c` | Stepper motor and sensors (RTOS) |
| `system_LPC17xx.c` | LPC1768 system initialization |
| `ThreeSensors_main.c` | Integration of all three sensors |
| `system_LPC17xx.s` | LPC1768 startup assembly file |

---

## Notes

- Both **bare-metal** and **RTOS-based** implementations are provided for comparison.
- Some files are experimental or unoptimized and included for reference.
- Ensure correct pin configuration as per the LPC1768 datasheet.

---

## License

This project is intended for academic and learning purposes.
