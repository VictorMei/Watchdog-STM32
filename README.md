# MPU6050 STM32 Project

STM32F103RB firmware for reading an MPU6050 over I2C, receiving target data over UART, and driving a pan servo with TIM3 PWM.

## Build

This project can be built with CMake and the ARM GNU toolchain:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

The generated firmware files are written to `build/Debug/` and are ignored by Git.

## Hardware

- STM32F103RB
- MPU6050 connected to I2C1
- UART2 for serial communication
- TIM3 channel 1 for pan-servo PWM
