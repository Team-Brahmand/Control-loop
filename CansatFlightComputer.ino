// CansatFlightComputer.ino

// SECTION: INCLUDES
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include "esp_task_wdt.h" // For the watchdog timer
#include "esp_system.h"   // For getting reset reason

// Include your custom configuration
#include "config.h"

// TODO: Add your specific sensor libraries here
#include <TinyGPS++.h>
// #include <Adafruit_LPS22.h>
// #include <Adafruit_BME680.h>
// ... etc ...


// SECTION: GLOBAL OBJECTS & VARIABLES
TinyGPSPlus gps;
// Adafruit_LPS22 lps22;
// Adafruit_BME680 bme680;

enum FlightState {
    BOOT, LAUNCH_PAD, ASCENT, DESCENT, RECOVERY
};

volatile struct SensorHealth {
    bool primary_baro_ok;
    bool gps_ok;
    bool imu_ok;
} sensorHealth;

volatile struct TelemetryData {
    // Required Fields
    unsigned long packetCount = 0;
    float timeStamp = 0.0;
    float altitude = 0.0;
    float pressure = 0.0;
    float temperature = 0.0;
    float voltage = 0.0;
    FlightState flightState = BOOT;
    float gnssLatitude = 0.0, gnssLongitude = 0.0, gnssAltitude = 0.0;
    int gnssSatellites = 0;
    float imu_accelX = 0.0, imu_accelY = 0.0, imu_accelZ = 0.0;
    float gyroSpinRate = 0.0;

    // Additional useful data
    float air_quality_metric_1 = 0.0; // Generic field for a primary air quality sensor
    float air_quality_metric_2 = 0.0; // Generic field for a secondary air quality sensor
} telemetryData;

volatile FlightState currentFlightState = BOOT;
TaskHandle_t Task_Core0, Task_Core1;
File dataFile, eventFile;
float lastAltitude = 0.0;


// SECTION: FUNCTION PROTOTYPES
void logEvent(String eventMessage);
void logDataToSD();
void resetWatchdogs();

// Core Task Functions
void core0_Task(void *pvParameters);
void core1_Task(void *pvParameters);

// Sensor and Redundancy Functions
void setupSensors();
void readAllSensors();
void readBarometer(); // Contains redundancy logic
void readGPS();
void readAirQualitySensors();
void readVoltage();

// State Machine and Actuators
void updateFlightState();
void manageActuators();

// Communication
void sendTelemetry();
void receiveCommands();

// ==============================================================================
// SETUP: Runs once on boot.
// ==============================================================================
void setup() {
    DEBUG_SERIAL.begin(115200);
    delay(1000); // Wait for serial to connect
    DEBUG_SERIAL.println("======================================");
    DEBUG_SERIAL.printf("CANSAT %s BOOTING...\n", TEAM_ID);
    
    // Check and log the reason for the last reset
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason != ESP_RST_POWERON) {
        logEvent("System rebooted. Reason code: " + String(reason));
    }

    // Initialize communications
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    GPS_SERIAL.begin(9600);
    XBEE_SERIAL.begin(9600);

    // Initialize SD Card
    if (!SD.begin(SD_CS_PIN)) {
        logEvent("CRITICAL: SD Card initialization failed!");
    } else {
        // Setup data log file with header
        dataFile = SD.open("/data_log.csv", FILE_APPEND);
        if (dataFile && dataFile.size() < 20) { // Only write header if file is new/empty
            dataFile.println("TeamID,Timestamp,Packet,State,Altitude,Pressure,Temp,Voltage,Lat,Lon,Sats,AccelZ,GyroSpin");
        }
        if(dataFile) dataFile.close();
        logEvent("SD Card Initialized. System Ready.");
    }
    
    setupSensors();
    pinMode(BUZZER_PIN, OUTPUT);
    
    // Configure and enable watchdog timers for both tasks
    esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
    esp_task_wdt_add(NULL); // Add the setup/loop task
    
    logEvent("Boot complete. Creating tasks.");
    currentFlightState = LAUNCH_PAD;
    
    // Create RTOS tasks
    xTaskCreatePinnedToCore(core0_Task, "Core0_Pilot", 10000, NULL, 2, &Task_Core0, 0);
    xTaskCreatePinnedToCore(core1_Task, "Core1_Comms", 10000, NULL, 1, &Task_Core1, 1);
}

void loop() {
    // The main Arduino loop is not used. We feed the watchdog and yield.
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ==============================================================================
// CORE 0 TASK: SENSORS, STATE MACHINE, LOGGING, ACTUATORS
// ==============================================================================
void core0_Task(void *pvParameters) {
    esp_task_wdt_add(NULL); // Register this task with the watchdog
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CORE_0_LOOP_RATE_HZ);
    
    for (;;) {
        esp_task_wdt_reset(); // Feed the watchdog
        readAllSensors();
        updateFlightState();
        manageActuators();
        logDataToSD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================================================
// CORE 1 TASK: TELEMETRY & COMMANDS
// ==============================================================================
void core1_Task(void *pvParameters) {
    esp_task_wdt_add(NULL); // Register this task with the watchdog
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CORE_1_LOOP_RATE_HZ);

    for (;;) {
        esp_task_wdt_reset(); // Feed the watchdog
        sendTelemetry();
        receiveCommands();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*********************************************************************************
 * HELPER FUNCTIONS
 *********************************************************************************/

// --- LOGGING ---
void logEvent(String eventMessage) {
    String logEntry = String(millis() / 1000.0) + "s: " + eventMessage;
    DEBUG_SERIAL.println("EVENT: " + logEntry);
    eventFile = SD.open("/events.log", FILE_APPEND);
    if (eventFile) {
        eventFile.println(logEntry);
        eventFile.close();
    }
}

void logDataToSD() {
    dataFile = SD.open("/data_log.csv", FILE_APPEND);
    if (dataFile) {
        char dataString[200];
        // Format to match header: TeamID,Timestamp,Packet,State,Altitude,Pressure,Temp,Voltage,Lat,Lon,Sats,AccelZ,GyroSpin
        snprintf(dataString, sizeof(dataString), "%s,%.2f,%lu,%d,%.1f,%.0f,%.1f,%.2f,%.4f,%.4f,%d,%.2f,%.2f",
            TEAM_ID, telemetryData.timeStamp, telemetryData.packetCount, telemetryData.flightState,
            telemetryData.altitude, telemetryData.pressure, telemetryData.temperature, telemetryData.voltage,
            telemetryData.gnssLatitude, telemetryData.gnssLongitude, telemetryData.gnssSatellites,
            telemetryData.imu_accelZ, telemetryData.gyroSpinRate);
        dataFile.println(dataString);
        dataFile.close();
    } // If file fails to open, do nothing and continue the mission.
}


// --- SENSORS & REDUNDANCY ---
void setupSensors() {
    logEvent("Initializing sensors...");
    // TODO: Add sensor init code. Update health status on success/fail.
    // if (!lps22.begin_I2C()) {
    //     logEvent("ERROR: Primary Baro (LPS22) failed to init.");
    //     sensorHealth.primary_baro_ok = false;
    //     // Try to init backup sensor
    //     if (!bme680.begin()) { logEvent("ERROR: Backup Baro (BME680) also failed."); }
    // } else {
    //     sensorHealth.primary_baro_ok = true;
    // }
}

void readAllSensors() {
    telemetryData.timeStamp = millis() / 1000.0f;
    readBarometer();
    readGPS();
    readAirQualitySensors();
    readVoltage();
    // TODO: Call your other sensor read functions (IMU, etc.)
}

void readBarometer() {
    // This function contains redundancy logic.
    // It tries the primary sensor first, then the backup.
    // TODO: Fill with your sensor library code
    if (sensorHealth.primary_baro_ok) {
        // telemetryData.pressure = lps22.readPressure();
        // telemetryData.temperature = lps22.readTemperature();
    } else {
        // Primary failed, try backup
        // telemetryData.pressure = bme680.pressure / 100.0F; // Pa
        // telemetryData.temperature = bme680.temperature;
    }
    // Calculate altitude from pressure
    // telemetryData.altitude = 44330 * (1.0f - pow(telemetryData.pressure / (SEALEVELPRESSURE_HPA * 100), 0.1903));
}

void readGPS() {
    while (GPS_SERIAL.available() > 0) {
        if (gps.encode(GPS_SERIAL.read())) {
            if (gps.location.isValid()) {
                telemetryData.gnssLatitude = gps.location.lat();
                telemetryData.gnssLongitude = gps.location.lng();
                telemetryData.gnssAltitude = gps.altitude.meters();
                telemetryData.gnssSatellites = gps.satellites.value();
            }
        }
    }
}

void readAirQualitySensors() {
    // Generic placeholder for your air quality sensors.
    // TODO: Add code to read from your Prana Air, DFRobot, etc. sensors
    // and populate the generic telemetry fields.
    // Example:
    // telemetryData.air_quality_metric_1 = readPM25_concentration();
    // telemetryData.air_quality_metric_2 = readOzone_ppm();
}

void readVoltage() {
    int rawADC = analogRead(VOLTAGE_PIN);
    telemetryData.voltage = (rawADC / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
}


// --- STATE MACHINE & ACTUATORS ---
void updateFlightState() {
    FlightState previousState = currentFlightState;
    float vertical_velocity = (telemetryData.altitude - lastAltitude) / (1.0 / CORE_0_LOOP_RATE_HZ);
    
    switch (currentFlightState) {
        case LAUNCH_PAD:
            if (telemetryData.imu_accelZ > LAUNCH_ACCELERATION_THRESHOLD_G) {
                currentFlightState = ASCENT;
            }
            break;
        case ASCENT:
            if (telemetryData.altitude < (lastAltitude - APOGEE_ALTITUDE_DROP_M)) {
                currentFlightState = DESCENT;
            }
            break;
        case DESCENT:
            if (abs(vertical_velocity) < LANDING_SPEED_THRESHOLD_MPS) {
                // Could be landing, start a timer or counter
            }
            break;
        case RECOVERY:
            // Stay in this state
            break;
    }
    lastAltitude = telemetryData.altitude;
    
    if (currentFlightState != previousState) {
        logEvent("State change: " + String(previousState) + " -> " + String(currentFlightState));
    }
    telemetryData.flightState = currentFlightState;
}

void manageActuators() {
    switch(currentFlightState) {
        case DESCENT:
            // TODO: Logic to deploy second descent mechanism at 500m
            if (telemetryData.altitude <= 500) {
                // Example: myServo.write(90);
            }
            break;
        case RECOVERY:
            // Activate recovery beacon
            digitalWrite(BUZZER_PIN, HIGH);
            break;
        default:
            // Ensure actuators are off
            digitalWrite(BUZZER_PIN, LOW);
            break;
    }
}


// --- COMMUNICATION ---
void sendTelemetry() {
    telemetryData.packetCount++;
    char telemetryString[256];
    
    // Format required by CANSAT India guidelines [cite: 316]
    // <TEAM ID>,<TIME STAMPING>,<PACKET COUNT>,<ALTITUDE>,<PRESSURE>,<TEMP>,<VOLTAGE>,
    // <GNSS LATITUDE>,<GNSS LONGITUDE>,<GNSS ALTITUDE>,<GNSS SATS>,
    // <ACCELEROMETER DATA>,<GYRO SPIN RATE>,<FLIGHT SOFTWARE STATE>
    snprintf(telemetryString, sizeof(telemetryString),
             "%s,%.2f,%lu,%.1f,%.0f,%.1f,%.2f,%.4f,%.4f,%.1f,%d,%.2f,%.2f,%d\r\n",
             TEAM_ID, telemetryData.timeStamp, telemetryData.packetCount,
             telemetryData.altitude, telemetryData.pressure, telemetryData.temperature, telemetryData.voltage,
             telemetryData.gnssLatitude, telemetryData.gnssLongitude, telemetryData.gnssAltitude, telemetryData.gnssSatellites,
             telemetryData.imu_accelZ, // Using Z-axis as the primary accelerometer data example
             telemetryData.gyroSpinRate,
             telemetryData.flightState);

    XBEE_SERIAL.print(telemetryString);
    DEBUG_SERIAL.print("TX-> " + String(telemetryString));
}

void receiveCommands() {
    if (XBEE_SERIAL.available()) {
        String command = XBEE_SERIAL.readStringUntil('\n');
        command.trim();
        logEvent("Command received: " + command);

        if (command.equalsIgnoreCase("CALIBRATE")) {
            // TODO: Add sensor calibration logic here
            logEvent("Executing calibration command.");
        } else if (command.equalsIgnoreCase("START_TX")) {
            // In this code, transmission starts automatically, but this can be a manual override
            logEvent("Telemetry transmission confirmed.");
        } else {
            logEvent("Unknown command.");
        }
    }
}
