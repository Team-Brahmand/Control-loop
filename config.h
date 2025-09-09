// config.h

#ifndef CONFIG_H
#define CONFIG_H

/*********************************************************************************
 * CANSAT USER CONFIGURATION
 *********************************************************************************/

// --- Team and CANSAT Identification ---
const char* TEAM_ID = "2024ASI-CANSAT-123"; // REQUIRED: Set your unique Team ID

// --- Hardware Pin Definitions ---
// VERIFY these pins match your PCB layout. These are for a generic ESP32 DevKit.
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_SCK_PIN  18
#define SD_CS_PIN    5

#define BUZZER_PIN     26
#define SERVO_PIN      27
#define N20_MOTOR_PIN1 32
#define N20_MOTOR_PIN2 33
#define VOLTAGE_PIN    34

// --- Serial Port Configuration ---
// Using hardware serial ports for reliability
#define DEBUG_SERIAL Serial      // For USB debugging output
#define XBEE_SERIAL  Serial2   // For XBEE communication
#define GPS_SERIAL   Serial1   // For GPS module

// --- Task Control & Watchdog Timers ---
#define CORE_0_LOOP_RATE_HZ 20
#define CORE_1_LOOP_RATE_HZ 1
#define WATCHDOG_TIMEOUT_S  5 // Seconds before watchdog triggers a reset

// --- Flight State Machine Thresholds ---
// These values are EXAMPLES. You MUST tune them through testing.
#define LAUNCH_ACCELERATION_THRESHOLD_G 5.0f // G-force to detect launch
#define APOGEE_ALTITUDE_DROP_M        5.0f // Meters of altitude drop to detect apogee
#define LANDING_SPEED_THRESHOLD_MPS   0.5f // m/s to detect landing
#define LANDING_TIMEOUT_MS            5000 // Milliseconds of low speed to confirm landing

// --- Sensor Configuration ---
#define SEALEVELPRESSURE_HPA (1013.25) // Standard pressure at sea level
#define VOLTAGE_DIVIDER_RATIO 2.0f     // Set based on your voltage divider resistors (R1+R2)/R2

#endif // CONFIG_H
