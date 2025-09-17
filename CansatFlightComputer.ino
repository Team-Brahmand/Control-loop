// CansatFlightComputer.ino

// SECTION: INCLUDES
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include "esp_system.h"
#include "config.h"
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

// Sensor Objects
Adafruit_LPS22 lps = Adafruit_LPS22();
Adafruit_Sensor *lps_pressure = NULL;
Adafruit_Sensor *lps_temp = NULL;
TinyGPSPlus gps;

// SECTION: GLOBAL OBJECTS & VARIABLES
enum FlightState {
    BOOT, LAUNCH_PAD, ASCENT, DESCENT, RECOVERY
};

volatile bool isInTestMode = false;
bool sdCardOK = false;

volatile struct TelemetryData {
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
    float air_quality_metric_1 = 0.0;
    float air_quality_metric_2 = 0.0;
} telemetryData;

volatile FlightState currentFlightState = BOOT;
TaskHandle_t Task_Core0, Task_Core1;
File dataFile, eventFile;

// SECTION: FUNCTION PROTOTYPES
void logEvent(String eventMessage);
void logDataToSD();
void core0_Task(void *pvParameters);
void core1_Task(void *pvParameters);
void setupSensors();
void readAllSensors();
void readBarometer();
void readGPS();
void readIMU();
void readAirQualitySensors();
void readVoltage();
void updateFlightState();
void manageActuators();
void sendTelemetry();
void receiveCommands();
void processCommand(String command);

// ==============================================================================
// SETUP: Runs once on boot.
// ==============================================================================
void setup() {
    DEBUG_SERIAL.begin(115200);
    delay(1000);

    // LED Startup Sequence
    pinMode(STATUS_LED_PIN, OUTPUT);
    unsigned long startTime = millis();
    while (millis() - startTime < 3000) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }

    DEBUG_SERIAL.println("======================================");
    DEBUG_SERIAL.printf("CANSAT %s BOOTING...\n", TEAM_ID);
    
    isInTestMode = false;

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    XBEE_SERIAL.begin(9600);
    // Remap Serial1 to free pins if default RX1/TX1 are unavailable
    // Format: begin(baud, config, RX_PIN, TX_PIN)
    GPS_SERIAL.begin(9600, SERIAL_8N1, 13, 12);

    if (!SD.begin(SD_CS_PIN, SPI)) {
        sdCardOK = false;
        logEvent("CRITICAL: SD Card initialization failed!");
    } else {
        sdCardOK = true;
        dataFile = SD.open("/data_log.csv", FILE_APPEND);
        if (dataFile && dataFile.size() < 20) {
            dataFile.println("TeamID,Timestamp,Packet,State,Altitude,Pressure,Temp,Voltage,Lat,Lon,Sats,AccelZ,GyroSpin");
        }
        if (dataFile) dataFile.close();

        eventFile = SD.open("/events.csv", FILE_APPEND);
        if (eventFile && eventFile.size() < 20) {
            eventFile.println("Timestamp,Event");
        }
        if (eventFile) eventFile.close();
        
        logEvent("SD Card Initialized. System Ready.");
    }
    
    setupSensors();
    pinMode(BUZZER_PIN, OUTPUT);
    
    logEvent("System booted. Defaulting to FLIGHT MODE.");
    logEvent("Boot complete. Creating tasks.");
    currentFlightState = LAUNCH_PAD;
    
    xTaskCreatePinnedToCore(core0_Task, "Core0_Pilot", 10000, NULL, 2, &Task_Core0, 0);
    xTaskCreatePinnedToCore(core1_Task, "Core1_Comms", 10000, NULL, 1, &Task_Core1, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void core0_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CORE_0_LOOP_RATE_HZ);
    static bool ledState = false;
    
    for (;;) {
        // "Breathing" LED indicator for normal operation
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState);

        readAllSensors();
        updateFlightState();
        manageActuators();
        logDataToSD();
        vTaskDelay(pdMS_TO_TICKS(1)); // Stability delay
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void core1_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CORE_1_LOOP_RATE_HZ);

    for (;;) {
        sendTelemetry();
        receiveCommands();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void logEvent(String eventMessage) {
    String logEntryCSV = String(millis() / 1000.0, 2) + "," + eventMessage;
    String logEntryHuman = "EVENT: " + String(millis() / 1000.0, 2) + "s: " + eventMessage;
    DEBUG_SERIAL.println(logEntryHuman);
    if (sdCardOK) {
        eventFile = SD.open("/events.csv", FILE_APPEND);
        if (eventFile) {
            eventFile.println(logEntryCSV);
            eventFile.close();
        }
    }
}

void logDataToSD() {
    if (!sdCardOK) {
        return;
    }
    dataFile = SD.open("/data_log.csv", FILE_APPEND);
    if (dataFile) {
        char dataString[200];
        snprintf(dataString, sizeof(dataString), "%s,%.2f,%lu,%d,%.2f,%.0f,%.2f,%.2f,%.4f,%.4f,%d,%.2f,%.2f",
            TEAM_ID, telemetryData.timeStamp, telemetryData.packetCount, telemetryData.flightState,
            telemetryData.altitude, telemetryData.pressure, telemetryData.temperature, telemetryData.voltage,
            telemetryData.gnssLatitude, telemetryData.gnssLongitude, telemetryData.gnssSatellites,
            telemetryData.imu_accelZ, telemetryData.gyroSpinRate);
        dataFile.println(dataString);
        dataFile.close();
    }
}

void setupSensors() {
    logEvent("Initializing sensors...");
    if (!lps.begin_I2C()) {
        logEvent("CRITICAL ERROR: Failed to find LPS22 sensor!");
        while (1) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            DEBUG_SERIAL.println("LPS22 not found! Check wiring. Halting...");
            delay(1000);
        }
    } else {
        logEvent("LPS22 Sensor Initialized.");
        lps_pressure = lps.getPressureSensor();
        lps_temp = lps.getTemperatureSensor();
    }
}

void readAllSensors() {
    telemetryData.timeStamp = millis() / 1000.0f;
    readBarometer();
    readGPS();
    readIMU();
    readAirQualitySensors();
    readVoltage();
    #if DEBUG_PRINT_ENABLED
        if (!isInTestMode) {
            DEBUG_SERIAL.println();
        }
    #endif
}

void readBarometer() {
    if (isInTestMode) {
        telemetryData.pressure = 101300.0 + (rand() % 100 - 50);
        telemetryData.temperature = 25.0 + (float)(rand() % 100) / 100.0;
        telemetryData.altitude = 44330 * (1.0f - pow(telemetryData.pressure / (SEALEVELPRESSURE_HPA * 100), 0.1903));
    } else {
        sensors_event_t pressure_event;
        sensors_event_t temp_event;
        lps_pressure->getEvent(&pressure_event);
        lps_temp->getEvent(&temp_event);

        telemetryData.pressure = pressure_event.pressure * 100;
        telemetryData.temperature = temp_event.temperature;
        
        #if DEBUG_PRINT_ENABLED
            DEBUG_SERIAL.printf("LPS22 -> Pressure: %.2f Pa, Temp: %.2f C\n", telemetryData.pressure, telemetryData.temperature);
        #endif

        if (telemetryData.pressure > 0) {
            telemetryData.altitude = 44330 * (1.0f - pow(telemetryData.pressure / (SEALEVELPRESSURE_HPA * 100), 0.1903));
        }
    }
}

void readGPS() {
    if (isInTestMode) {
        telemetryData.gnssLatitude = 17.3850 + (float)(rand() % 100) / 10000.0;
        telemetryData.gnssLongitude = 78.4867 + (float)(rand() % 100) / 10000.0;
        telemetryData.gnssAltitude = 545.0 + (float)(rand() % 20);
        telemetryData.gnssSatellites = (rand() % 4) + 7;
    } else {
        while (GPS_SERIAL.available() > 0) {
            gps.encode(GPS_SERIAL.read());
        }

        if (gps.location.isUpdated() && gps.location.isValid()) {
            telemetryData.gnssLatitude = gps.location.lat();
            telemetryData.gnssLongitude = gps.location.lng();
            telemetryData.gnssAltitude = gps.altitude.meters();
            telemetryData.gnssSatellites = gps.satellites.value();

            #if DEBUG_PRINT_ENABLED
                DEBUG_SERIAL.printf("GPS -> Lat: %.4f, Lon: %.4f, Sats: %d\n", 
                                    telemetryData.gnssLatitude, 
                                    telemetryData.gnssLongitude, 
                                    telemetryData.gnssSatellites);
            #endif
        }
    }
}

void readIMU() {
    if (isInTestMode) {
        telemetryData.imu_accelZ = 1.0 + (float)(rand() % 50) / 100.0;
        telemetryData.gyroSpinRate = 5.0 + (float)(rand() % 200) / 100.0;
    }
}

void readAirQualitySensors() {
    if (isInTestMode) {
        telemetryData.air_quality_metric_1 = 40.0 + (float)(rand() % 10);
        telemetryData.air_quality_metric_2 = 0.05 + (float)(rand() % 10) / 100.0;
    }
}

void readVoltage() {
    if (isInTestMode) {
        telemetryData.voltage = 4.1 - (float)(rand() % 20) / 100.0;
    } else {
        int rawADC = analogRead(VOLTAGE_PIN);
        telemetryData.voltage = (rawADC / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
        
        #if DEBUG_PRINT_ENABLED
            DEBUG_SERIAL.printf("Voltage -> Batt: %.2f V\n", telemetryData.voltage);
        #endif
    }
}

void updateFlightState() {
    if (isInTestMode) {
        currentFlightState = LAUNCH_PAD;
    }
}
void manageActuators() {
    if (isInTestMode) {
        digitalWrite(BUZZER_PIN, LOW);
    }
}

void sendTelemetry() {
    telemetryData.packetCount = telemetryData.packetCount + 1;
    char telemetryString[256];
    
    snprintf(telemetryString, sizeof(telemetryString),
             "%s,%.2f,%lu,%d,%.2f,%.0f,%.2f,%.2f,%.4f,%.4f,%.1f,%d,%.2f,%.2f,%d\r\n",
             TEAM_ID, telemetryData.timeStamp, telemetryData.packetCount,
             telemetryData.altitude, telemetryData.pressure, telemetryData.temperature, telemetryData.voltage,
             telemetryData.gnssLatitude, telemetryData.gnssLongitude, telemetryData.gnssAltitude, telemetryData.gnssSatellites,
             telemetryData.imu_accelZ,
             telemetryData.gyroSpinRate,
             telemetryData.flightState);

    XBEE_SERIAL.print(telemetryString);
    DEBUG_SERIAL.print("TX-> " + String(telemetryString));
}

void processCommand(String command) {
    command.trim();
    if (command.length() == 0) return;

    logEvent("Command received: " + command);

    if (command.equalsIgnoreCase("CMD,SET_MODE,TEST")) {
        if (!isInTestMode) {
            logEvent("SWITCHING TO TEST MODE.");
            isInTestMode = true;
        }
    } else if (command.equalsIgnoreCase("CMD,SET_MODE,FLIGHT")) {
        if (isInTestMode) {
            logEvent("SWITCHING TO FLIGHT MODE.");
            isInTestMode = false;
        }
    } else if (command.equalsIgnoreCase("CALIBRATE")) {
        logEvent("Executing calibration command.");
    } else {
        logEvent("Unknown command: " + command);
    }
}

void receiveCommands() {
    if (DEBUG_SERIAL.available()) {
        String command = DEBUG_SERIAL.readStringUntil('\n');
        processCommand(command);
    }
    if (XBEE_SERIAL.available()) {
        String command = XBEE_SERIAL.readStringUntil('\n');
        processCommand(command);
    }
}
