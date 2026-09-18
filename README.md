# Watchdog Car — STM32 Firmware

This is the STM32 half of the Watchdog car: a pan/tilt camera platform on a
two-wheel chassis that visually tracks a person and can optionally drive
toward them.

A Raspberry Pi (or laptop) runs the vision side — it detects and selects a
target with YOLO and streams the target's position over UART. This
repository is the firmware that receives that data and actually moves the
servos and motors.

**Vision code (runs on the Pi/laptop):** https://github.com/tshiven/Watchdog_car

## What it does

- Two RC servos (pan + tilt) keep the camera pointed at the tracked target.
- A 16x2 LCD and two status LEDs show whether a target is currently seen
  and/or locked.
- Two DC motors (driven through an L9110S module) can optionally drive the
  chassis toward the target, using the same vision data.
- If the vision link drops or the target is lost, everything freezes /
  stops rather than guessing.

The two things vision sends over UART are:
1. A high-rate **tracking packet** — how far off-centre the target is,
   whether it's detected/locked, and roughly how big it is in frame
   (used as a distance estimate).
2. A low-rate **target-name packet** — the label of whichever target is
   currently selected, shown on the LCD.

## Hardware

- STM32F103RB ("Blue Pill" / Nucleo-style board)
- 2x RC servo — pan (TIM3 CH1 / PA6) and tilt (TIM3 CH2 / PA7)
- L9110S dual motor driver — 2 DC motors for the drivetrain (plain GPIO,
  no PWM speed control)
- USART2 (PA2/PA3) — serial link to the vision computer, 115200 baud
- 16x2 HD44780 LCD over a PCF8574 I2C backpack, on I2C1 (remapped to
  PB8/PB9)
- 2 status LEDs — blue (PA10, "detected") and green (PB3, "locked")
- MPU6050 IMU is wired to the same I2C1 bus but is not currently used by
  the firmware (see `lcd_i2c.c`'s notes) — it's reserved for a future
  stabilisation feature

## Where to start reading

If you're getting familiar with this codebase, read in this order:

1. **`Core/Src/main.c`** — this is almost the whole project. Everything
   that isn't boilerplate lives here: the UART packet parser, the pan/tilt
   control loop, the autonomous drivetrain logic, and the LED/LCD status
   code. It's heavily commented — the comments explain *why* each constant
   has the value it does, not just what the code does.
2. **`Core/Src/lcd_i2c.c`** / **`Core/Inc/lcd_i2c.h`** — a small, standalone
   driver for the status LCD. Doesn't know anything about tracking; `main.c`
   just tells it what text to show.
3. **`Core/Src/stm32f1xx_hal_msp.c`** — pin/clock setup for each peripheral
   (which GPIO pins map to which peripheral function). Useful if you're
   rewiring something.
4. **`Core/Src/stm32f1xx_it.c`** — interrupt vector table entries. The only
   one that matters for this project is `USART2_IRQHandler`, which is what
   actually triggers the byte-by-byte UART parsing in `main.c`.
5. Everything else (`system_stm32f1xx.c`, `syscalls.c`, `sysmem.c`,
   `stm32f1xx_hal_conf.h`, `Core/Startup/*`, `Drivers/`, `Middlewares/`) is
   CubeMX-generated or vendor HAL/CMSIS code. You generally won't need to
   touch it.

## Project structure

```
Core/
  Inc/
    main.h                  CubeMX-generated pin/peripheral declarations
    lcd_i2c.h                LCD driver interface
    stm32f1xx_it.h            Interrupt handler declarations
    stm32f1xx_hal_conf.h      HAL feature configuration
  Src/
    main.c                   Tracking, drivetrain, UART protocol - the actual robot logic
    lcd_i2c.c                LCD driver implementation
    stm32f1xx_it.c            Interrupt handlers
    stm32f1xx_hal_msp.c       Peripheral pin/clock setup
    system_stm32f1xx.c        CubeMX clock startup code
    syscalls.c / sysmem.c     Minimal libc stubs needed to link
  Startup/
    startup_stm32f103xb.s     Reset vector / startup assembly
Drivers/                     ST's HAL and CMSIS libraries (vendor code)
mpu6050_v1.ioc                STM32CubeMX project file (pin/clock config)
CMakeLists.txt, CMakePresets.json, cmake/   Build configuration
STM32F103*_FLASH.ld           Linker scripts
```

## Build

This project is built with CMake and the ARM GNU toolchain:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Generated firmware files go to `build/Debug/` (git-ignored).

## Related repository

The vision side of this project — target detection, selection, and the
Python program that talks to this firmware over serial — lives in a
separate repo: **https://github.com/tshiven/Watchdog_car**
