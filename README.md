STM32 UART Environmental Data Logger using FreeRTOS
Overview

This project implements a real-time environmental data logger on the STM32F407G-DISC1 development board using FreeRTOS. The firmware periodically acquires temperature, pressure and humidity data from a BME280 sensor together with the current time from a DS1307 RTC, combines both datasets, and transmits them over UART to a host PC.

The project was developed using a Makefile-based build system and a command-line development workflow without relying on STM32CubeIDE for building, flashing or debugging.

Features
FreeRTOS based firmware
Periodic environmental data acquisition
Real-time clock integration
UART based data logging
Interactive UART command interface for setting RTC time
Event-driven task synchronization using FreeRTOS notifications
Queue based inter-task communication
Command-line build, flashing and debugging
Python automation scripts for firmware development
Hardware
Development Board
STM32F407G-DISC1
System Clock: 16 MHz
Sensors
BME280 (I2C2)
DS1307 RTC (I2C1)
Communication
USART2
FT232 USB-UART converter
Peripheral Usage
Peripheral	Purpose
USART2	UART communication with host PC
I2C1	DS1307 RTC
I2C2	BME280 environmental sensor
TIM6	HAL time base (FreeRTOS uses SysTick)
LED Status Indicators
LED	Function
Red	DS1307 initialized successfully
Blue	BME280 initialized successfully
Orange	Command mode exited successfully
FreeRTOS Configuration
configTICK_RATE_HZ = 1000
Preemptive Scheduler
Tick Period = 1 ms
Tasks

The firmware consists of six FreeRTOS tasks.

Initialization Tasks

These tasks execute only once during system startup.

RTC initialization
BME280 initialization
Runtime Tasks
UART Task

Responsible for

Receiving user commands
Formatting outgoing log messages
UART transmission
RTC Task

Responsible for

Reading current time from DS1307
Writing new RTC time after receiving a valid command
BME280 Task

Responsible for

Triggering measurements (Forced Mode)
Reading temperature
Reading pressure
Reading humidity
Command Task

Responsible for

Processing user entered commands
Validating RTC time format
Passing processed data to the RTC task
Inter-task Communication

The firmware uses

Indexed task notifications
ISR-safe task notifications
Three FreeRTOS queues

Queues are used for

RTC timestamp
BME280 sensor data
User command

The UART task waits until both RTC and BME280 data become available before formatting and transmitting the combined log.

UART Output

Example

11:35:12 -- 29.3C 1002hPa 62%
11:35:13 -- 29.4C 1002hPa 62%
11:35:14 -- 29.4C 1001hPa 61%

Configure the serial terminal with

Baud Rate: (your baud rate)
Local Echo: ON

The project has been tested using Tera Term.

Setting RTC Time

Enter command mode by pressing

>

Terminate every command using

#
24 Hour Format
HH:MM:SS#

Example

18:45:30#
12 Hour Format
HH:MM:SS AM#

or

HH:MM:SS PM#

Example

08:15:20 PM#

Commands not matching the required format are ignored.

Build and Flash Automation

Python helper scripts are provided.

Build
python3 build.py

Builds the firmware executable

final.elf
Flash and Debug
python3 flash_debug.py

This script

Starts OpenOCD
Connects GDB
Downloads the firmware
Leaves the user inside an interactive GDB session for debugging

Prerequisites

OpenOCD
gdb-multiarch
Flash and Run
python3 flash_run.py

Downloads the firmware, starts execution on the target and exits.

Build System

The project uses

GNU Make
arm-none-eabi-gcc
OpenOCD
gdb-multiarch
Python

No IDE is required for building or flashing.

Future Improvements
Automatic UART log capture to CSV
Data visualization using Python (Matplotlib)
CI-based automated firmware builds
Unit testing of host-side Python utilities
RTOS-aware debugging support
Project Structure
.
├── Core/
├── Drivers/
├── ThirdParty/
├── build.py
├── flash_debug.py
├── flash_run.py
├── Makefile
└── final.elf
