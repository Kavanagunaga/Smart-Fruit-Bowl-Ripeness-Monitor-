
# Smart Fruit Bowl – Ripeness Monitor 🍌

A low-power IoT-based smart fruit bowl that monitors fruit ripeness using gas resistance, temperature, and humidity measurements from a Bosch BME680 sensor connected to an XIAO ESP32-S3.

The system collects sensor data, sends measurements through MQTT, stores data across deep-sleep cycles using RTC memory, and classifies fruit condition based on changes in gas resistance and humidity.

---

## 📌 Project Overview

Fruit ripening is associated with changes in volatile organic compounds (VOCs), temperature, and humidity.

This project develops a smart fruit bowl that uses the BME680 gas sensor to monitor these changes and estimate the ripeness condition of fruit.

The system is designed as a low-power IoT node using:

- XIAO ESP32-S3
- Bosch BME680 sensor
- I²C communication
- Wi-Fi
- MQTT
- ESP-IDF
- FreeRTOS
- RTC memory
- Deep sleep

The sensor measures temperature, humidity, pressure, and gas resistance. The gas resistance variation is used as the primary indicator for ripeness classification.

---

## 🎯 Objectives

- Monitor fruit-related gas resistance changes.
- Measure temperature and humidity.
- Estimate fruit ripeness condition.
- Send sensor data through MQTT.
- Store measurements across deep-sleep cycles.
- Reduce power consumption using ESP32-S3 deep sleep.
- Implement an automated ripeness classification algorithm.
- Log sensor and decision data for analysis.

---

## 🧠 System Concept

```text
                 ┌──────────────────────┐
                 │      BME680 Sensor   │
                 │                      │
                 │ Temperature          │
                 │ Humidity             │
                 │ Pressure             │
                 │ Gas Resistance       │
                 └──────────┬───────────┘
                            │
                           I²C
                            │
                            ▼
                 ┌──────────────────────┐
                 │   XIAO ESP32-S3      │
                 │                      │
                 │ Sensor Processing    │
                 │ Ring Buffer          │
                 │ Ripeness Algorithm   │
                 │ Power Management     │
                 └──────────┬───────────┘
                            │
                     Wi-Fi / MQTT
                            │
                            ▼
                 ┌──────────────────────┐
                 │     MQTT Broker      │
                 │   broker.emqx.io     │
                 └──────────┬───────────┘
                            │
                            ▼
                 ┌──────────────────────┐
                 │   Python MQTT Logger │
                 │                      │
                 │ CSV Data Logging     │
                 │ Decision Logging     │
                 └──────────────────────┘
