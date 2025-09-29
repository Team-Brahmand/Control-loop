// config.h

#ifndef CONFIG_H
#define CONFIG_H

/*********************************************************************************
 * CANSAT USER CONFIGURATION
 *********************************************************************************/

// --- System Mode ---
#define DEBUG_PRINT_ENABLED true // Set to true for detailed sensor prints, false for clean output

// --- Team and CANSAT Identification ---
const char* TEAM_ID = "2024ASI-CANSAT-123";

// --- Hardware Pin Definitions ---
// I2C Pins (Shared by LPS22, SHT41, BME680, ADS1115)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// SPI Pins (for SD Card)
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_SCK_PIN  18
#define SD_CS_PIN    5

// Actuator & Indicator Pins
#define STATUS_LED_PIN 2  // Built-in LED on most ESP32 boards
#define BUZZER_PIN     26
#define SERVO_PIN      27
#define N20_MOTOR_PIN1 32
#define N20_MOTOR_PIN2 33

// Analog Input Pins
#define VOLTAGE_PIN    34

#define SD_CS_PIN 5

// --- Serial Port Configuration ---
#define DEBUG_SERIAL Serial
#define XBEE_SERIAL  Serial2
#define GPS_SERIAL   Serial1 // Using Serial1 for the NEO-M8N

// --- Task Control ---
#define CORE_0_LOOP_RATE_HZ 0.5f // Main sensor loop runs twice per second
#define CORE_1_LOOP_RATE_HZ 1.0f // Comms loop runs once per second

// --- Flight State Machine Thresholds (Example Values) ---
#define LAUNCH_ACCELERATION_THRESHOLD_G 5.0f
#define APOGEE_ALTITUDE_DROP_M        5.0f
#define LANDING_SPEED_THRESHOLD_MPS   0.5f

// --- Sensor Configuration ---
#define SEALEVELPRESSURE_HPA (1013.25)
#define VOLTAGE_DIVIDER_RATIO 2.0f

#endif // CONFIG_H
