# Esp32-Wilcoxon-883M
Firmware for Industrial IoT Vibration Monitoring using ESP32 and Wilcoxon 883M sensors.

# 🏭 Industrial IoT Vibration Monitor (ESP32 + Wilcoxon 883M)

![License](https://img.shields.io/badge/license-MIT-green)
![Platform](https://img.shields.io/badge/platform-ESP32-blue)
![Framework](https://img.shields.io/badge/framework-Arduino-orange)
![Version](https://img.shields.io/badge/version-3.11.0-blueviolet)
![Status](https://img.shields.io/badge/status-Production%20Ready-success)

> **A professional-grade IoT gateway firmware for monitoring industrial machinery vibration using the Wilcoxon 883M (RS485) sensor and the Cytron Maker Feather AIoT S3.**

---

## 📋 Project Overview

This firmware transforms a standard Modbus RTU vibration sensor into a modern **Industrial IoT Node**. It provides real-time monitoring of critical machinery health metrics (Velocity, Acceleration, Displacement) via a web dashboard, MQTT, and Modbus TCP.

### ✨ Key Features (v3.11.0)
* **📊 Real-Time Web Dashboard:** Hosted directly on the ESP32. Visualizes live metrics and high-frequency waveform/spectrum graphs using Chart.js.
* **☁️ MQTT Integration:** Publishes health status, metrics, and raw data chunks to any MQTT broker (TLS supported).
* **🔗 Modbus TCP Gateway:** Acts as a bridge, allowing SCADA systems (Ignition, Node-RED) to query the sensor over Wi-Fi.
* **🆔 UUID & Identity:** Auto-detects sensor Serial Number, Firmware Version, and Unique Device UUID.
* **🕹️ PLC Remote Trigger:** New register allows PLCs to trigger data captures remotely.
* **🚨 Smart Alerts:** Local buzzer alarms and LED indicators for Wi-Fi loss or sensor errors.
* **💾 CSV Export:** Download internal metric logs directly from the browser.

---

## 🛠️ Hardware Requirements

| Component | Description | Notes |
| :--- | :--- | :--- |
| **MCU** | [Cytron Maker Feather AIoT S3](https://www.cytron.io/) | ESP32-S3 with built-in RGB LED & Buzzer |
| **Sensor** | [Wilcoxon 883M](https://wilcoxon.com/) | Digital Vibration Sensor (RS485 Modbus RTU) |
| **Power** | 12V-24V DC PSU | Required to power the 883M sensor |
| **Module** | RS485-to-TTL UART | For ESP32 to communicate with the sensor |

### 🔌 Pin Configuration
| ESP32 Pin | Function |
| :--- | :--- |
| **GPIO 16** | RS485 RX |
| **GPIO 15** | RS485 TX |
| **GPIO 12** | Buzzer (Alarm) |
| **GPIO 3** | User Button (Hold 5s for AP Mode) |
| **GPIO 46** | RGB Status LED |

---

## 🚀 Installation & Setup

### 1. Flash the Firmware
You can use **PlatformIO** (recommended) or Arduino IDE.
1.  Clone this repository.
2.  Open the folder in VS Code with the PlatformIO extension installed.
3.  Connect your ESP32 via USB.
4.  Click **Upload** (➡️).

### 2. Connect to Wi-Fi
1.  On first boot, the device will create a Wi-Fi Access Point:
    * **SSID:** `ILA_Sensor_AP`
    * **Password:** `12345678`
2.  Connect your phone/laptop to this network.
3.  Open a browser to `http://192.168.4.1`.
4.  Go to **Settings > Wi-Fi** to scan and save your local network credentials.

---

## 📡 API & Communication

### 🟢 Status LED Codes
* 🟢 **Green:** System Healthy (Idle)
* 🟡 **Yellow (Blinking):** Sensor Busy (Downloading Waveform)
* 🔴 **Red:** Sensor Error / Timeout
* ⚪ **White:** Wi-Fi Disconnected
* 🔵 **Cyan:** Metrics Recording Active

### 🏭 Modbus TCP Server Map
The ESP32 acts as a Modbus TCP Slave on **Port 502**. This allows SCADA/PLC systems to read cached data.

#### 1. Scalar Metrics (Standard Input Registers)
Read these registers to get the latest processed values.

| Reg | Description | Scale | Unit |
| :--- | :--- | :--- | :--- |
| **0** | X Accel RMS | x100 | g |
| **1** | X Accel Peak | x100 | g |
| **2** | X Accel RMS (10-5k Hz) | x100 | g |
| **3** | X Accel Peak (10-5k Hz) | x100 | g |
| **4** | X Velocity RMS | x10 | mm/s |
| **5** | X Velocity Peak | x10 | mm/s |
| **6** | X Displacement RMS | x1 | µm |
| **7** | X Displacement Peak | x1 | µm |
| **8** | X True Peak | x100 | g |
| **9** | X Crest Factor | x100 | - |
| **20** | Y Accel RMS | x100 | g |
| **40** | Z Accel RMS | x100 | g |
| **60** | Temperature | x10 | °C |
| **95** | **PLC Trigger Command** | Write `1` | Triggers a new capture |

#### 2. Large Data Access (Waveform & Spectrum)
The Gateway maps the latest captured Waveform and Spectrum data to direct address ranges for fast reading.

| Reg Start | Size | Content | Format |
| :--- | :--- | :--- | :--- |
| **1000** | 13,334 | Time Waveform (Latest Axis) | Int16 (x0.001 g) |
| **20000** | 6,145 | FFT Spectrum (Latest Axis) | UInt16 (x0.001 g) |

---

## ☁️ MQTT Topics & Commands

The device connects to your configured MQTT Broker (supports TLS/SSL) and interacts via JSON payloads.

**Default Base Topic:** `883M` (Configurable in Settings)

### 📤 Published Topics (Device -> Cloud)

| Topic | Payload Type | Description |
| :--- | :--- | :--- |
| `883M/status` | JSON | **Health Check:** Publishes every 5 minutes.<br>Contains: Uptime, RSSI, IP, Serial Number, **UUID**, Free RAM. |
| `883M` | JSON | **Metrics:** Object containing scalar values (RMS, Peak, Temp).<br>**Waveform:** Chunked arrays of raw time data.<br>**Spectrum:** Chunked arrays of frequency data. |

### 📥 Command Topic (Cloud -> Device)

**Topic:** `883M/command`

> **⚠️ CRITICAL REQUIREMENT:**
> Every command **MUST** include the `serial_number` field matching the device's specific serial number.

#### 1. Trigger Data Capture (Single Axis)
```json
{
  "serial_number": "ENTER_DEVICE_SERIAL_HERE",
  "command": "trigger_capture",
  "axis": "X"
}
```
(Valid axis values: "X", "Y", "Z")

### 2. Set Auto-Capture Interval
Updates how often the device automatically cycles through X, Y, and Z axes.

```json

{
  "serial_number": "ENTER_DEVICE_SERIAL_HERE",
  "command": "set_interval",
  "interval_ms": 600000
}
(Example: 600000 = 10 Minutes. Set to 2147483647 to disable auto-capture.)
```

### 3. Remote Reboot
Forces the ESP32 to restart.

```json

{
  "serial_number": "ENTER_DEVICE_SERIAL_HERE",
  "command": "reboot"
}
```


### 4. Request Immediate Metrics
Forces the device to publish the latest scalar metrics (RMS/Peak) to MQTT immediately without waiting for the next interval.

```json
{
  "serial_number": "ENTER_DEVICE_SERIAL_HERE",
  "command": "get_metrics"
}
```
