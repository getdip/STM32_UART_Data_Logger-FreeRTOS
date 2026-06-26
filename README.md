Markdown
# STM32 UART Environmental Data Logger (FreeRTOS)

A real-time environmental data logger built on the **STM32F407G-DISC1** using **FreeRTOS**. The firmware periodically acquires environmental data from a BME280 sensor, timestamps it using a DS1307 RTC, and streams the combined data over USART2 to a host PC.

---

## Features

* **FreeRTOS Architecture:** Event-driven firmware using FreeRTOS tasks.
* **Periodic Logging:** Environmental data logging at a strict 1-second interval.
* **Sensor Handling:** BME280 operating efficiently in *Forced Mode*.
* **Time Tracking:** Integrated DS1307 Real-Time Clock (RTC).
* **Command Interface:** UART command interface to dynamically set/update the RTC time.
* **Task Synchronization:** Uses FreeRTOS task notifications (including indexed notifications) and Queue-based inter-task communication.
* **Modern Build System:** Makefile-based build system wrapped with Python automation for building, flashing, and debugging.
* **CLI Debugging:** OpenOCD + GDB command-line debugging integration.

---

## Hardware Configuration

* **Development Board:** STM32F407G-DISC1
* **MCU Clock:** 16 MHz
* **Sensors & Peripherals:**
  * **BME280:** Connected via `I2C2`
  * **DS1307 RTC:** Connected via `I2C1`
  * **UART:** `USART2` routed through an FT232 USB-UART Adapter to the host PC

---

### System Clock
* `SystemCoreClock = 16000000;` (16 MHz)

### FreeRTOS Settings
* `configTICK_RATE_HZ = 1000`
* **Scheduler:** Preemptive
* **Tick Period:** 1 ms

### HAL Time Base
* `TIM6` is dedicated as the HAL time base, as `SysTick` is exclusively reserved by FreeRTOS.

---

## FreeRTOS Tasks

| Task | Purpose |
| :--- | :--- |
| **RTC Init** | Initializes the DS1307 RTC peripheral |
| **BME Init** | Initializes the BME280 sensor |
| **UART Task** | Handles all UART Tx/Rx operations |
| **RTC Task** | Manages periodic RTC read and write operations |
| **BME Task** | Manages sensor acquisition and scheduling |
| **CMD Task** | Processes user commands incoming via UART |

---

## Inter-Task Communication

### Task Notifications
Used extensively across the application for:
* Peripheral initialization handshakes
* Sensor synchronization
* UART control flow
* ISR-to-task synchronization

### Queues
Three dedicated FreeRTOS queues handle data safety:
1. **RTC Queue:** Stores timestamp data.
2. **BME280 Queue:** Stores environmental sensor data.
3. **User Command Queue:** Stores raw incoming CLI commands.

---

## Data Flow

1. **RTC Task** reads the current timestamp every second.
2. **BME280 Task** triggers a Forced Mode conversion and reads the raw data.
3. Both tasks push their respective datasets into their dedicated **Queues**.
4. **UART Task** blocks until both datasets become available in the queues.
5. Once data is fetched, the UART Task formats the string and transmits it over **USART2**.
6. The **FT232 Adapter** forwards this data directly to the host PC's terminal.

---

## UART Command Interface

### How to Use
* **Enter Command Mode:** Transmit the `>` character.
* **Termination:** Every command must be terminated with a `#` character.

### Command Formats

* **24-Hour Format:** `HH:MM:SS#`
  * *Example:* `18:45:30#`
* **12-Hour Format:** `HH:MM:SS AM#` or `HH:MM:SS PM#`
  * *Example:* `08:30:15 PM#`

> **Note:** Any commands failing to match these strict formats will be safely ignored by the command parser.

---

## LED Indicators

| LED | Status | Description |
| :--- | :--- | :--- |
| 🔴 **Red** | ON | DS1307 initialized successfully |
| 🔵 **Blue** | ON | BME280 initialized successfully |
| 🟠 **Orange** | ON | Command mode entered (System in cmd state) |
| 🟠 **Orange** | OFF | Command mode exited (System in Idle state) |

---

## Build, Flash & Run

### 1. Build the Project
```bash
python3 build.py
```
### 2. Flash and run
```bash
python3 flash_run.py --target final.elf
```
### 3. Flash and enter debug mode(GDB through openOCD)
```bash
python3 flash_debug.py --target final.elf
```
