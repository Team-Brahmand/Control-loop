# Cansat Flight Computer

This repository contains the firmware for a CanSat flight computer built using an ESP32. It handles sensor readings, SD card logging, GPS data, LoRa telemetry, and flight-state detection.

## Features
- Reads data from:
  - **BME680** (pressure, temperature, gas)
  - **SHT41** (humidity and temperature)
  - **ICM-20948 IMU** (accel, gyro, mag)
  - **Ublox GPS** module
- Logs all data to **SD card** in CSV format
- Sends live telemetry using **LoRa**
- **OLED display** for debugging
- Flight state machine:
  - Pre-flight
  - Ascent
  - Descent
  - Landing


## Hardware Required
- ESP32 Dev Module  
- BME680  
- SHT41  
- ICM-20948 IMU  
- Ublox GPS  
- LoRa SX1276/78  
- SD card module  
- OLED (SSD1306)

## How to Use
1. Open the `.ino` file in Arduino IDE or PlatformIO.  
2. Install required libraries:
   - Adafruit BME680  
   - SparkFun ICM-20948  
   - TinyGPS++  
   - LoRa  
   - Adafruit SSD1306  
3. Select **ESP32 Dev Module**.  
4. Upload the code.  
5. Insert SD card and power on the device.

## CSV Log Format
time, state, altitude, pressure, temperature, voltage, gps_lat, gps_lon, gps_alt, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, humidity, gas_resistance
