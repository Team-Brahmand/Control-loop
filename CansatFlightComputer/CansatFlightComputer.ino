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

#include "Adafruit_SHT4x.h"
#include <Adafruit_ADS1X15.h>
#include <bsec.h> // Bosch BSEC library for BME680

// SECTION: SENSOR OBJECTS
Adafruit_LPS22 lps = Adafruit_LPS22();
Adafruit_Sensor *lps_pressure = NULL;
Adafruit_Sensor *lps_temp = NULL;
TinyGPSPlus gps;
Adafruit_SHT4x sht41 = Adafruit_SHT4x();
// Adafruit_ADS1115 ads;
Bsec bme680; // BSEC library object for BME680

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
    
    // SHT41 Data
    float sht41_temperature = 0.0;
    float sht41_humidity = 0.0;

    // BME680 Data (Redundancy + Air Quality)
    float bme680_pressure = 0.0;
    float bme680_temperature = 0.0;
    float bme680_humidity = 0.0;
    float bme680_gas_resistance = 0.0; // In Ohms
    float bme680_iaq = 0.0; // Index for Air Quality

    // ADS1115 Data (Payload Sensor)
    float payload_sensor_voltage = 0.0;

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
void readVoltage();
void readSHT41();
void readBME680();
void readADS1115();
void checkIaqSensorStatus(void);
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
    Wire.setClock(100000); // Set I2C clock to 100kHz (standard speed)
    XBEE_SERIAL.begin(9600);
    GPS_SERIAL.begin(9600, SERIAL_8N1, 13, 12); // RX on 13, TX on 12

    if (!SD.begin(SD_CS_PIN, SPI)) {
        sdCardOK = false;
        logEvent("CRITICAL: SD Card initialization failed!");
    } else {
        sdCardOK = true;
        dataFile = SD.open("/data_log.csv", FILE_APPEND);
        if (dataFile && dataFile.size() < 20) {
            dataFile.println("TeamID,Timestamp,Packet,State,Altitude,Pressure,Temp,Voltage,Lat,Lon,Sats,AccelZ,GyroSpin,SHT_Temp,SHT_Hum,BME_Press,BME_Temp,BME_Hum,BME_Gas,BME_IAQ,ADS_V");
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

// ==============================================================================
// CORE 0 TASK: Handles sensors, flight logic, and data logging.
// ==============================================================================
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

// ==============================================================================
// CORE 1 TASK: Handles telemetry transmission and command reception.
// ==============================================================================
void core1_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CORE_1_LOOP_RATE_HZ);

    for (;;) {
        sendTelemetry();
        receiveCommands();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================================================
// DATA LOGGING FUNCTIONS
// ==============================================================================
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
        char dataString[350]; // Increased buffer size for new data
        snprintf(dataString, sizeof(dataString), "%s,%.2f,%lu,%d,%.2f,%.0f,%.2f,%.2f,%.4f,%.4f,%d,%.2f,%.2f,%.2f,%.2f,%.0f,%.2f,%.2f,%.0f,%.1f,%.3f",
            TEAM_ID, telemetryData.timeStamp, telemetryData.packetCount, telemetryData.flightState,
            telemetryData.altitude, telemetryData.pressure, telemetryData.temperature, telemetryData.voltage,
            telemetryData.gnssLatitude, telemetryData.gnssLongitude, telemetryData.gnssSatellites,
            telemetryData.imu_accelZ, telemetryData.gyroSpinRate,
            telemetryData.sht41_temperature, telemetryData.sht41_humidity,
            telemetryData.bme680_pressure, telemetryData.bme680_temperature, telemetryData.bme680_humidity,
            telemetryData.bme680_gas_resistance, telemetryData.bme680_iaq,
            telemetryData.payload_sensor_voltage);
        dataFile.println(dataString);
        dataFile.close();
    }
}

// ==============================================================================
// SENSOR SETUP AND READING FUNCTIONS
// ==============================================================================
void setupSensors() {
    logEvent("Initializing sensors...");
    // LPS22
    if (!lps.begin_I2C()) {
        logEvent("CRITICAL ERROR: Failed to find LPS22 sensor!");
        while (1) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(1000);
        }
    } else {
        logEvent("LPS22 Sensor Initialized.");
        lps_pressure = lps.getPressureSensor();
        lps_temp = lps.getTemperatureSensor();
    }

    // SHT41
    if (!sht41.begin()) {
        logEvent("ERROR: Failed to find SHT41 sensor!");
    } else {
        logEvent("SHT41 Sensor Initialized.");
        sht41.setPrecision(SHT4X_HIGH_PRECISION);
        sht41.setHeater(SHT4X_NO_HEATER);
    }

    // // ADS1115
    // if (!ads.begin()) {
    //     logEvent("ERROR: Failed to find ADS1115!");
    // } else {
    //     logEvent("ADS1115 ADC Initialized.");
    // }
    
    // BME680 with BSEC
    // CHANGED: Use BME68X_I2C_ADDR_LOW for the primary address 0x76
    bme680.begin(BME68X_I2C_ADDR_LOW, Wire); 
    logEvent("BME680 BSEC version " + String(bme680.version.major) + "." + String(bme680.version.minor) + "." + String(bme680.version.major_bugfix) + "." + String(bme680.version.minor_bugfix));
    checkIaqSensorStatus();
    
    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE, BSEC_OUTPUT_RAW_PRESSURE, BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS, BSEC_OUTPUT_IAQ, BSEC_OUTPUT_STATIC_IAQ, BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, BSEC_OUTPUT_STABILIZATION_STATUS, BSEC_OUTPUT_RUN_IN_STATUS
    };
    bme680.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();
}

void readAllSensors() {
    telemetryData.timeStamp = millis() / 1000.0f;
    readBarometer();
    readGPS();
    readIMU();
    readVoltage();
    readSHT41();
    readBME680();
    // readADS1115();

    #if DEBUG_PRINT_ENABLED
        if (!isInTestMode) {
            DEBUG_SERIAL.println("--------------------------------------");
        }
    #endif
}

void readBarometer() { // LPS22
    if (isInTestMode) {
        telemetryData.pressure = 101300.0 + (rand() % 100 - 50);
        telemetryData.temperature = 25.0 + (float)(rand() % 100) / 100.0;
    } else {
        sensors_event_t pressure_event, temp_event;
        lps_pressure->getEvent(&pressure_event);
        lps_temp->getEvent(&temp_event);
        telemetryData.pressure = pressure_event.pressure * 100; // hPa to Pa
        telemetryData.temperature = temp_event.temperature;
        #if DEBUG_PRINT_ENABLED
            DEBUG_SERIAL.printf("LPS22 -> Pressure: %.2f Pa, Temp: %.2f C\n", telemetryData.pressure, telemetryData.temperature);
        #endif
    }
    // Calculate altitude based on the primary pressure sensor
    if (telemetryData.pressure > 0) {
        telemetryData.altitude = 44330 * (1.0f - pow(telemetryData.pressure / (SEALEVELPRESSURE_HPA * 100), 0.1903));
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
                DEBUG_SERIAL.printf("GPS -> Lat: %.4f, Lon: %.4f, Sats: %d\n", telemetryData.gnssLatitude, telemetryData.gnssLongitude, telemetryData.gnssSatellites);
            #endif
        }
    }
}

void readIMU() {
    // NOTE: This is a stub function. You need to add your IMU library (e.g., ICM-20948, MPU-6050)
    // and reading logic here.
    if (isInTestMode) {
        telemetryData.imu_accelZ = 1.0 + (float)(rand() % 50) / 100.0;
        telemetryData.gyroSpinRate = 5.0 + (float)(rand() % 200) / 100.0;
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

void readSHT41() {
    if (isInTestMode) {
        telemetryData.sht41_temperature = 24.5 + (float)(rand() % 100) / 100.0;
        telemetryData.sht41_humidity = 55.0 + (float)(rand() % 100) / 100.0;
    } else {
        sensors_event_t humidity, temp;
        sht41.getEvent(&humidity, &temp);
        telemetryData.sht41_temperature = temp.temperature;
        telemetryData.sht41_humidity = humidity.relative_humidity;
        #if DEBUG_PRINT_ENABLED
            DEBUG_SERIAL.printf("SHT41 -> Temp: %.2f C, Humidity: %.2f %%\n", telemetryData.sht41_temperature, telemetryData.sht41_humidity);
        #endif
    }
}

void readBME680() {
    if (isInTestMode) {
        telemetryData.bme680_pressure = 101350.0 + (rand() % 100 - 50);
        telemetryData.bme680_temperature = 26.0 + (float)(rand() % 100) / 100.0;
        telemetryData.bme680_humidity = 54.0 + (float)(rand() % 100) / 100.0;
        telemetryData.bme680_gas_resistance = 50000.0 + (rand() % 1000);
        telemetryData.bme680_iaq = 25.0 + (rand() % 10);
    } else {
        if (bme680.run()) { // This reads the sensor
            telemetryData.bme680_pressure = bme680.pressure;
            telemetryData.bme680_temperature = bme680.temperature;
            telemetryData.bme680_humidity = bme680.humidity;
            telemetryData.bme680_gas_resistance = bme680.gasResistance;
            telemetryData.bme680_iaq = bme680.iaq;
            #if DEBUG_PRINT_ENABLED
                DEBUG_SERIAL.printf("BME680 -> Press: %.0f Pa, Temp: %.2f C, Hum: %.2f %%, Gas: %.0f Ohm, IAQ: %.1f\n", 
                    telemetryData.bme680_pressure, telemetryData.bme680_temperature, telemetryData.bme680_humidity,
                    telemetryData.bme680_gas_resistance, telemetryData.bme680_iaq);
            #endif
        } else {
            checkIaqSensorStatus();
        }
    }
}

// void readADS1115() {
//     if (isInTestMode) {
//         telemetryData.payload_sensor_voltage = 3.3 * (float)(rand() % 100) / 100.0;
//     } else {
//         int16_t adc0 = ads.readADC_SingleEnded(0);
//         // This factor (0.1875mV) is for the default gain setting (+/-6.144V range).
//         // Change if you set a different gain (e.g., ads.setGain(GAIN_ONE)).
//         telemetryData.payload_sensor_voltage = adc0 * 0.0001875F;
//         #if DEBUG_PRINT_ENABLED
//             DEBUG_SERIAL.printf("ADS1115 -> A0 Raw: %d, Voltage: %.3f V\n", adc0, telemetryData.payload_sensor_voltage);
//         #endif
//     }
// }

void checkIaqSensorStatus(void) {
    String output = "";
    // CHANGED: bme680.status is now bme680.bsecStatus
    if (bme680.bsecStatus != BSEC_OK) {
        if (bme680.bsecStatus < BSEC_OK) output = "BSEC error";
        else output = "BSEC warning";
        logEvent(output + ": " + String(bme680.bsecStatus));
    }
    
    // CHANGED: bme680.bme680Status is now bme680.bme68xStatus
    // CHANGED: BME680_OK is now BME68X_OK
    if (bme680.bme68xStatus != BME68X_OK) {
        if (bme680.bme68xStatus < BME68X_OK) output = "BME680 error";
        else output = "BME680 warning";
        logEvent(output + ": " + String(bme680.bme68xStatus));
    }
}

// ==============================================================================
// FLIGHT LOGIC & ACTUATORS (STUBS)
// ==============================================================================
void updateFlightState() {
    // NOTE: This is a stub function. You need to implement your flight state machine logic here,
    // using the thresholds defined in config.h.
    if (isInTestMode) {
        currentFlightState = LAUNCH_PAD;
    }
}

void manageActuators() {
    // NOTE: This is a stub function. You need to implement actuator control based on the currentFlightState.
    // (e.g., deploy parachute in DESCENT state, activate buzzer in RECOVERY state)
    if (isInTestMode) {
        digitalWrite(BUZZER_PIN, LOW);
    }
}

// ==============================================================================
// COMMUNICATION FUNCTIONS
// ==============================================================================

void sendTelemetry() {
    telemetryData.packetCount = telemetryData.packetCount + 1;
    char telemetryString[300]; // Increased buffer size
    
    // WARNING: This packet is very long. Ensure your ground station and radio can handle this length.
    // This example sends a subset of data for brevity. Customize as needed.
    // CORRECTED: Changed format specifiers (%d -> %.2f) for float values like altitude.
    snprintf(telemetryString, sizeof(telemetryString),
             "%s,%.2f,%lu,%d,%.2f,%.0f,%.2f,%.2f,%.4f,%.4f,%.1f,%d,%.2f,%.2f,%.2f,%.1f,%d\r\n",
             TEAM_ID, 
             telemetryData.timeStamp, 
             telemetryData.packetCount,
             telemetryData.flightState, // Note: State is now 4th, Altitude is 5th
             telemetryData.altitude,
             telemetryData.bme680_pressure, 
             telemetryData.bme680_temperature, 
             telemetryData.voltage,
             telemetryData.gnssLatitude, 
             telemetryData.gnssLongitude, 
             telemetryData.gnssAltitude, 
             telemetryData.gnssSatellites,
             telemetryData.imu_accelZ,
             telemetryData.gyroSpinRate,
             telemetryData.sht41_humidity,
             telemetryData.bme680_iaq,
             (int)telemetryData.flightState); // The old state variable at the end

    XBEE_SERIAL.print(telemetryString);
    if(DEBUG_PRINT_ENABLED){
        DEBUG_SERIAL.print("TX-> " + String(telemetryString));
    }
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
        // Add sensor calibration logic here
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
