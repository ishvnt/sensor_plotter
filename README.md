# sensor_plotter

**sensor_plotter** collects data from an MPU6050 sensor (via I2C) and a PS2 joystick (via ADC) using an STM32 microcontroller running FreeRTOS. The STM32 transmits this data over UART to an ESP32, which packages the data as JSON and sends it to a laptop using WebSockets. The laptop receives and plots the data in real time with a Python script using matplotlib.

## Project Structure

```
sensor_plotter/
├── esp32/     # ESP-IDF firmware for ESP32
├── stm32/     # STM32 firmware (CubeMX/IDE)
├── python/    # Python plotting script
├── requirements.txt/
└── README.md
```


## System Overview

| Component | Function |
| :-- | :-- |
| STM32 | STM32CubeIDE project implementing FreeRTOS tasks for I2C communication (MPU6050), ADC readings, and UART transmission |
| ESP32 | Receives UART data, packages as JSON, transmits via WebSocket |
| Laptop | Python script receives WebSocket data and plots it live |


## Getting Started

### 1. Hardware Requirements

- **STM32 MCU** (tested on STM32F446RE Nucleo Board)
- **ESP32 Dev Board**
- **MPU6050 sensor** (I2C)
- **PS2 Joystick** (Analog)
- **Wires, breadboard, USB cables**


### 2. Software Requirements

- **STM32CubeIDE** (for STM32 firmware)
- **ESP-IDF** (for ESP32 firmware)
- **Python 3.8+**
- **Python packages:** See `requirements.txt`


### 3. Building and Flashing

#### STM32

1. Open `stm32/` in STM32CubeIDE.
2. Connect sensors:
    - MPU6050 to I2C pins
    - PS2 Joystick to ADC pins
3. Connect ESP32 to UART pins.
4. Build and flash the firmware to the STM32 board.

#### ESP32

1. Open `esp32/` in VS Code with ESP-IDF extension.
2. Configure UART pins to match STM32 TX/RX.
3. Configure WiFi SSID and Password using sdkconfig.
3. Build and flash the firmware to ESP32.

#### Python Plotter

1. Setup and activate a virtual environment (venv).
```bash
python -m venv .venv
source .venv/bin/activate
```
2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Run the plotting script:

```bash
python plotter.py
```