# STM32F0 Discovery Board - Precharge Control for 48V Li-Ion Battery Pack

This project involves the implementation of a precharge control system for a 48V Li-Ion battery pack using an STM32F0 Discovery board. The system handles the precharge process to safely charge the capacitors in the power system, ensuring they are charged before the main contactor is closed. Additionally, the project includes VCU programs for controlling cabin lights and a buzzer that activates if the vehicle is driven with a door open.

## Features

- **Precharge Control**
  - Manages precharge of a 48V Li-Ion battery pack to ensure safe operation.
  - Controls precharge and main contactor relays based on voltage sensing.
- **Voltage Sensing**
  - Monitors supply and input voltages using ADC.
  - Utilizes filtering to remove noise and ensure accurate readings.
- **Safety Mechanisms**
  - Includes debounce logic for switches.
  - Implements timeout for precharge process.
  - Handles kill switch for emergency shutdown.
- **VCU Programs**
  - **Cabin Lights Driver**: Controls the cabin lights based on door switches.
  - **Buzzer Driver**: Activates a buzzer if the vehicle is driven with a door open.

## Functional Overview

### Precharge Control

- **Initialization**
  - Sets up GPIO pins, ADC, UART, and other peripherals.
  - Configures system clock and initializes SysTick for delay management.
- **Voltage Sensing**
  - Reads supply and input voltages through ADC channels.
  - Converts ADC values to voltages and averages them to remove outliers.
- **Control Logic**
  - Manages the state of precharge and main contactor relays.
  - Ensures safe precharge by checking voltage thresholds and timing.

### Safety Mechanisms

- **Debounce Logic**
  - Debounces various switches to prevent false triggering.
- **Kill Switch**
  - Monitors the kill switch to handle emergency shutdown.
- **Timeout Handling**
  - Implements timeouts for precharge process using RC time constants.

### VCU Programs

- **Cabin Lights Driver**
  - Controls the cabin lights based on the status of door switches.
  - Turns on the appropriate cabin lights when a door is open and off when closed.
- **Buzzer Driver**
  - Activates a buzzer if the vehicle is driven with any door open.
  - Ensures the buzzer is off when all doors are closed.

## How It Works

1. **Initialization**
   - Initializes the system clock, GPIOs, ADC, UART, and other peripherals.
   - Configures voltage source pins and initializes SysTick for timing.
2. **Voltage Sensing**
   - Continuously senses supply and input voltages.
   - Filters and averages the readings to ensure accurate values.
3. **Precharge Process**
   - Manages precharge relay based on voltage readings.
   - Engages main contactor once the precharge is complete.
4. **Safety Monitoring**
   - Monitors kill switch and handles emergency shutdown.
   - Implements debounce logic for various switches.
5. **Cabin Lights and Buzzer Control**
   - Controls cabin lights based on door switch status.
   - Activates buzzer if any door is open while the vehicle is in motion.

## Dependencies

- STM32F0 Discovery Board
- STM32CubeMX and STM32CubeIDE
- Peripheral drivers for GPIO, ADC, UART, and SysTick

## Getting Started

1. **Setup**
   - Connect the STM32F0 Discovery board to the 48V Li-Ion battery pack.
   - Ensure all connections are secure and correctly configured.
2. **Compile and Upload**
   - Compile the code using STM32CubeIDE.
   - Upload the code to the STM32F0 Discovery board.
3. **Run the System**
   - Power on the system and monitor the precharge process.
   - Check UART output for status messages and any errors.

## Project Structure

- **main.c**
  - Contains the main control logic for the precharge process.
  - Handles initialization, voltage sensing, control logic, and safety mechanisms.
  - Includes VCU programs for cabin lights and buzzer control.
- **stm32f0xx.h**
  - Header file for the STM32F0 series.
- **stm32f0xx_it.c**
  - Interrupt service routines.
- **system_stm32f0xx.c**
  - System initialization and configuration.

