Cansat Flight Computer

This repository contains the firmware for a CanSat flight computer based on the ESP32.
It handles sensor data acquisition, flight-state detection, SD logging, GPS, and LoRa telemetry.

Features

Reads data from BME680, SHT41, ICM-20948 (IMU), GPS (Ublox)

Stores data on SD card in CSV format

Sends telemetry via LoRa

Simple state machine for:

Pre-flight

Ascent

Descent

Landing

OLED display support for debugging

Structured logging with timestamps

Repository Structure
/CansatFlightComputer_Enhanced
│── CansatFlightComputer_Enhanced.ino   # Main firmware
│── /data/                              # Optional test logs
│── README.md                           # Project documentation

Hardware Used

ESP32 Dev Module

BME680 (Environmental Sensor)

SHT41 (Humidity + Temperature)

ICM-20948 (9-DoF IMU)

Ublox GPS

LoRa SX1276/78

SD Card Module

OLED (SSD1306)

How to Use

Open the .ino file in Arduino IDE / PlatformIO.

Install the required libraries (Adafruit BME680, SparkFun ICM-20948, TinyGPS++, LoRa, etc.).

Select the correct board: ESP32 Dev Module.

Flash to the ESP32.

Insert SD card → power → flight computer starts logging automatically.

Telemetry Format

CSV with fields such as:

time, state, altitude, pressure, temp, voltage, gps_lat, gps_lon, accel_x, gyro, pm25, humidity
