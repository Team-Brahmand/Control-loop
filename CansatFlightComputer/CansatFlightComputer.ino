// ==============================================================================
// CansatFlightComputer_Integrated.ino
//
// DESCRIPTION:
// This code merges the CansatFlightComputer structure with the IntegratedSensorHub.
// - Retains the dual-core FreeRTOS architecture for robust operation.
// - Integrates BNO08x IMU, BMP390 Barometer, and ADS1115 ADC.
// - Adds non-blocking servo motor control for actuator tasks.
// - Includes an SSD1306 OLED display for real-time onboard telemetry.
//
// ==============================================================================

// SECTION: INCLUDES
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include "esp_system.h"
#include "config.h"
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <bsec.h> // Bosch BSEC library for BME680

// Libraries from the second script
#include <ESP32Servo.h>         // For Servo Control
#include <Adafruit_LPS2X.h>
#include <Adafruit_BMP3XX.h>      // BMP390
#include <Adafruit_BNO08x.h>      // BNO08x
#include <Adafruit_GFX.h>         // For OLED
#include <Adafruit_SSD1306.h>     // For OLED
#include "Adafruit_SHT4x.h"
#include <Adafruit_ADS1X15.h>

// ==============================================================================
// HARDWARE CONFIGURATION
// (Addresses from your second code file have been added here)
// ==============================================================================
#define SERVO_PIN 27            
#define BNO08X_I2C_ADDR 0x4B
#define BMP_I2C_ADDR    0x77
#define ADS_ADDR        0x48
#define OLED_ADDR       0x3C
#define BME_ADDR        0x76    // <-- ADDED THIS
#define LPS22_ADDR      0x5C    // <-- ADDED THIS

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// SECTION: SENSOR & ACTUATOR OBJECTS
Adafruit_LPS22 lps = Adafruit_LPS22();
Adafruit_Sensor *lps_pressure = NULL;
Adafruit_Sensor *lps_temp = NULL;
TinyGPSPlus gps;
Adafruit_SHT4x sht41 = Adafruit_SHT4x();
Adafruit_ADS1115 ads;
Bsec bme680;
Servo myServo;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno = Adafruit_BNO08x();
sh2_SensorValue_t bnoSensorValue;

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
    
    // IMU Data from BNO08x
    float bno_qi=NAN, bno_qj=NAN, bno_qk=NAN, bno_qr=NAN;

    // SHT41 Data
    float sht41_temperature = 0.0;
    float sht41_humidity = 0.0;

    // BME680 Data (Redundancy + Air Quality)
    float bme680_pressure = 0.0;
    float bme680_temperature = 0.0;
    float bme680_humidity = 0.0;
    float bme680_gas_resistance = 0.0;
    float bme680_iaq = 0.0;

    // BMP390 Data (Redundancy)
    float bmp_temp=NAN, bmp_pressure=NAN;

    // ADS1115 Data (Payload Sensor)
    float payload_sensor_voltage = 0.0;

} telemetryData;

TaskHandle_t Task_Core0, Task_Core1;
File dataFile, eventFile;

// NEW: Boolean flags to track sensor status
bool oled_ok=false, lps_ok=false, bmp_ok=false, bno_ok=false;
bool sht_ok=false, ads_ok=false, bme_ok=false, servo_ok=false;

volatile FlightState currentFlightState = BOOT;

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
void readBMP390();
void readADS1115();
void checkIaqSensorStatus(void);
void updateFlightState();
void manageActuators();
void sendTelemetry();
void receiveCommands();
void processCommand(String command);
void displayTelemetry();

// ==============================================================================
// SETUP: Runs once on boot.
// ==============================================================================
void setup() {
    DEBUG_SERIAL.begin(115200);
    delay(1000);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH); // Turn on LED during setup

    DEBUG_SERIAL.println("======================================");
    DEBUG_SERIAL.printf("CANSAT %s BOOTING...\n", TEAM_ID);
    
    isInTestMode = false;

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // Use 400kHz I2C speed for faster sensor reads
    XBEE_SERIAL.begin(9600);
    GPS_SERIAL.begin(9600, SERIAL_8N1, 13, 12);

    if (!SD.begin(SD_CS_PIN, SPI)) {
        sdCardOK = false;
        logEvent("CRITICAL: SD Card initialization failed!");
    } else {
        sdCardOK = true;
        dataFile = SD.open("/data_log.csv", FILE_APPEND);
        if (dataFile && dataFile.size() < 20) {
            // Updated header with new sensor data
            dataFile.println("TeamID,Timestamp,Packet,State,Altitude,Pressure,Temp,Voltage,Lat,Lon,Sats,BNO_qi,BNO_qj,BNO_qk,BNO_qr,SHT_Temp,SHT_Hum,BME_Press,BME_Temp,BME_Hum,BME_Gas,BME_IAQ,BMP_Press,BMP_Temp,ADS_V");
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

    digitalWrite(STATUS_LED_PIN, LOW); // Turn off LED after setup
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ==============================================================================
// CORE 0 TASK: Handles sensors, flight logic, display, actuators, and logging.
// ==============================================================================
void core0_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CORE_0_LOOP_RATE_HZ);
    static bool ledState = false;
    
    for (;;) {
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState);

        readAllSensors();
        updateFlightState();
        manageActuators();
        logDataToSD();
        displayTelemetry(); // Update the OLED display
        
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
    if (!sdCardOK) return;
    
    dataFile = SD.open("/data_log.csv", FILE_APPEND);
    if (dataFile) {
        char dataString[500]; // Increased buffer for new data
        snprintf(dataString, sizeof(dataString), "%s,%.2f,%lu,%d,%.2f,%.0f,%.2f,%.2f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.0f,%.2f,%.2f,%.0f,%.1f,%.0f,%.2f,%.3f",
            TEAM_ID, telemetryData.timeStamp, telemetryData.packetCount, telemetryData.flightState,
            telemetryData.altitude, telemetryData.pressure, telemetryData.temperature, telemetryData.voltage,
            telemetryData.gnssLatitude, telemetryData.gnssLongitude, telemetryData.gnssSatellites,
            telemetryData.bno_qi, telemetryData.bno_qj, telemetryData.bno_qk, telemetryData.bno_qr, // New IMU data
            telemetryData.sht41_temperature, telemetryData.sht41_humidity,
            telemetryData.bme680_pressure, telemetryData.bme680_temperature, telemetryData.bme680_humidity,
            telemetryData.bme680_gas_resistance, telemetryData.bme680_iaq,
            telemetryData.bmp_pressure, telemetryData.bmp_temp, // New BMP390 data
            telemetryData.payload_sensor_voltage);
        dataFile.println(dataString);
        dataFile.close();
    }
}

// ==============================================================================
// SENSOR SETUP AND READING FUNCTIONS
// ==============================================================================
// ==============================================================================
// SENSOR SETUP AND READING FUNCTIONS (ROBUST VERSION)
// ==============================================================================
void setupSensors() {
    logEvent("Initializing sensors (robust check)...");

    // OLED Display
    DEBUG_SERIAL.print("Initializing OLED... ");
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0);
        display.println("Sensors Init...");
        display.display();
        oled_ok = true;
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }

    // Servo
    DEBUG_SERIAL.print("Initializing Servo... ");
    ESP32PWM::allocateTimer(0);
    if (myServo.attach(SERVO_PIN)) {
        servo_ok = true;
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }

    // LPS22 (Primary Barometer)
    DEBUG_SERIAL.print("Initializing LPS22 (0x5C)... ");
    if (lps.begin_I2C(LPS22_ADDR)) {
        lps_pressure = lps.getPressureSensor();
        lps_temp = lps.getTemperatureSensor();
        lps_ok = true;
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }
    
    // BMP390 (Secondary Barometer)
    DEBUG_SERIAL.print("Initializing BMP390 (0x77)... ");
    if (bmp.begin_I2C(BMP_I2C_ADDR)) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp_ok = true;
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }
    
    // BNO08x (IMU)
    DEBUG_SERIAL.print("Initializing BNO08x (0x4B)... ");
    if (bno.begin_I2C(BNO08X_I2C_ADDR)) {
        if (bno.enableReport(SH2_ROTATION_VECTOR)) {
            bno_ok = true;
            DEBUG_SERIAL.println("OK");
        } else {
            DEBUG_SERIAL.println("FAILED to enable report!");
        }
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }

    // SHT41 (Humidity)
    DEBUG_SERIAL.print("Initializing SHT41... ");
    if (sht41.begin()) {
        sht_ok = true;
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }

    // ADS1115 (ADC)
    DEBUG_SERIAL.print("Initializing ADS1115 (0x48)... ");
    if (ads.begin(ADS_ADDR)) {
        ads.setGain(GAIN_ONE);
        ads_ok = true;
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }
    
    // BME680 (Air Quality)
    DEBUG_SERIAL.print("Initializing BME680 (0x76)... ");
    bme680.begin(BME_ADDR, Wire); 
    if (bme680.bsecStatus == BSEC_OK && bme680.bme68xStatus == BME68X_OK) {
        bsec_virtual_sensor_t sensorList[10] = { BSEC_OUTPUT_RAW_TEMPERATURE, BSEC_OUTPUT_RAW_PRESSURE, BSEC_OUTPUT_RAW_HUMIDITY, BSEC_OUTPUT_RAW_GAS, BSEC_OUTPUT_IAQ, BSEC_OUTPUT_STATIC_IAQ, BSEC_OUTPUT_CO2_EQUIVALENT, BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, BSEC_OUTPUT_STABILIZATION_STATUS, BSEC_OUTPUT_RUN_IN_STATUS };
        bme680.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
        bme_ok = true;
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }
    logEvent("Sensor init check complete.");
}

void readAllSensors() {
    telemetryData.timeStamp = millis() / 1000.0f;
    
    if (lps_ok) readBarometer();
    if (bno_ok) readIMU();
    if (sht_ok) readSHT41();
    if (bme_ok) readBME680();
    if (bmp_ok) readBMP390();
    if (ads_ok) readADS1115();
    
    // These functions don't depend on I2C sensors
    readGPS();
    readVoltage();
}

void readBarometer() { // LPS22
    sensors_event_t pressure_event, temp_event;
    lps_pressure->getEvent(&pressure_event);
    lps_temp->getEvent(&temp_event);
    telemetryData.pressure = pressure_event.pressure * 100; // hPa to Pa
    telemetryData.temperature = temp_event.temperature;
    
    // Calculate altitude based on the primary pressure sensor
    if (telemetryData.pressure > 0) {
        telemetryData.altitude = 44330 * (1.0f - pow(telemetryData.pressure / (SEALEVELPRESSURE_HPA * 100), 0.1903));
    }
}

void readGPS() {
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }
    if (gps.location.isUpdated() && gps.location.isValid()) {
        telemetryData.gnssLatitude = gps.location.lat();
        telemetryData.gnssLongitude = gps.location.lng();
        telemetryData.gnssAltitude = gps.altitude.meters();
        telemetryData.gnssSatellites = gps.satellites.value();
    }
}

void readIMU() { // Reads BNO08x
    if (bno.getSensorEvent(&bnoSensorValue)) {
      if (bnoSensorValue.sensorId == SH2_ROTATION_VECTOR) {
        telemetryData.bno_qi = bnoSensorValue.un.rotationVector.i;
        telemetryData.bno_qj = bnoSensorValue.un.rotationVector.j;
        telemetryData.bno_qk = bnoSensorValue.un.rotationVector.k;
        telemetryData.bno_qr = bnoSensorValue.un.rotationVector.real;
      }
    }
}

void readVoltage() {
    int rawADC = analogRead(VOLTAGE_PIN);
    telemetryData.voltage = (rawADC / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
}

void readSHT41() {
    sensors_event_t humidity, temp;
    sht41.getEvent(&humidity, &temp);
    telemetryData.sht41_temperature = temp.temperature;
    telemetryData.sht41_humidity = humidity.relative_humidity;
}

void readBME680() {
    if (bme680.run()) { // This reads the sensor
        telemetryData.bme680_pressure = bme680.pressure;
        telemetryData.bme680_temperature = bme680.temperature;
        telemetryData.bme680_humidity = bme680.humidity;
        telemetryData.bme680_gas_resistance = bme680.gasResistance;
        telemetryData.bme680_iaq = bme680.iaq;
    } else {
        checkIaqSensorStatus();
    }
}

void readBMP390() { // Reads secondary barometer
    if (bmp.performReading()) {
        telemetryData.bmp_temp = bmp.temperature;
        telemetryData.bmp_pressure = bmp.pressure;
    }
}

void readADS1115() {
    int16_t adc0 = ads.readADC_SingleEnded(0);
    // LSB value for GAIN_ONE (+/-4.096V) is 0.125mV
    telemetryData.payload_sensor_voltage = adc0 * 0.000125F;
}

void checkIaqSensorStatus(void) {
    String output = "";
    if (bme680.bsecStatus != BSEC_OK) {
        if (bme680.bsecStatus < BSEC_OK) output = "BSEC error";
        else output = "BSEC warning";
        logEvent(output + ": " + String(bme680.bsecStatus));
    }
    
    if (bme680.bme68xStatus != BME68X_OK) {
        if (bme680.bme68xStatus < BME68X_OK) output = "BME680 error";
        else output = "BME680 warning";
        logEvent(output + ": " + String(bme680.bme68xStatus));
    }
}

// ==============================================================================
// OLED DISPLAY FUNCTION
// ==============================================================================
void displayTelemetry() {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    
    display.printf("State: %d Alt: %.1fm\n", telemetryData.flightState, telemetryData.altitude);
    display.printf("P: %.0fhPa T: %.1fC\n", telemetryData.pressure / 100.0f, telemetryData.temperature);
    display.printf("Lat: %.3f\nLon: %.3f Sats:%d\n", telemetryData.gnssLatitude, telemetryData.gnssLongitude, telemetryData.gnssSatellites);
    display.printf("V: %.2fV IAQ: %.0f\n", telemetryData.voltage, telemetryData.bme680_iaq);
    display.printf("qi:%.2f qj:%.2f\n", telemetryData.bno_qi, telemetryData.bno_qj);
    display.printf("qk:%.2f qr:%.2f\n", telemetryData.bno_qk, telemetryData.bno_qr);

    display.display();
}

// ==============================================================================
// FLIGHT LOGIC & ACTUATORS
// ==============================================================================
void updateFlightState() {
    // NOTE: This is a stub function. Implement your flight state machine logic here.
    if (isInTestMode) {
        currentFlightState = LAUNCH_PAD;
    }
}

// void manageActuators() {
//     // This function now contains the NON-BLOCKING servo sweep logic.
//     // It can be expanded to control other things like buzzers or pyro channels.
    
//     static int servoPos = 0;
//     static int servoState = 0; // 0=sweeping up, 1=pausing at top, 2=sweeping down, 3=pausing at bottom
//     static unsigned long lastServoMoveTime = 0;

//     const int SWEEP_DELAY_MS = 15;
//     const int PAUSE_DELAY_MS = 2000;

//     if (millis() - lastServoMoveTime > SWEEP_DELAY_MS && (servoState == 0 || servoState == 2)) {
//         lastServoMoveTime = millis();
//         if (servoState == 0) { // Sweeping up
//             servoPos++;
//             myServo.write(servoPos);
//             if (servoPos >= 180) {
//                 servoState = 1; // Switch to pausing at top
//             }
//         } else { // Sweeping down
//             servoPos--;
//             myServo.write(servoPos);
//             if (servoPos <= 0) {
//                 servoState = 3; // Switch to pausing at bottom
//             }
//         }
//     }

//     if (millis() - lastServoMoveTime > PAUSE_DELAY_MS && (servoState == 1 || servoState == 3)) {
//         lastServoMoveTime = millis();
//         if (servoState == 1) { // Was pausing at top
//             servoState = 2; // Switch to sweeping down
//         } else { // Was pausing at bottom
//             servoState = 0; // Switch to sweeping up
//         }
//     }

//     // Example of other actuator logic
//     if (currentFlightState == RECOVERY) {
//         // activate buzzer
//     }
// }

void manageActuators() {
    // Actuator logic is now driven by the current flight state.
    // Static variables are used for the sweep logic so they retain their values.
    static int servoPos = 0;
    static int servoState = 0; // 0=sweeping up, 1=pausing at top, 2=sweeping down, 3=pausing at bottom
    static unsigned long lastServoMoveTime = 0;

    switch (currentFlightState) {
        case BOOT:
        case LAUNCH_PAD:
            // On the launch pad, hold the servo in its initial, safe position.
            // This check prevents sending the command repeatedly.
            if (myServo.read() != 0) {
                myServo.write(0);
            }
            break;

        case ASCENT: {
            // During ascent, run the continuous sweep logic for testing or control surfaces.
            const int SWEEP_DELAY_MS = 15;
            const int PAUSE_DELAY_MS = 2000;

            if (millis() - lastServoMoveTime > SWEEP_DELAY_MS && (servoState == 0 || servoState == 2)) {
                lastServoMoveTime = millis();
                if (servoState == 0) { // Sweeping up
                    servoPos++;
                    if (servoPos >= 180) {
                        servoPos = 180;
                        servoState = 1; // Switch to pausing at top
                    }
                } else { // Sweeping down
                    servoPos--;
                    if (servoPos <= 0) {
                        servoPos = 0;
                        servoState = 3; // Switch to pausing at bottom
                    }
                }
                myServo.write(servoPos);
            }

            if (millis() - lastServoMoveTime > PAUSE_DELAY_MS && (servoState == 1 || servoState == 3)) {
                lastServoMoveTime = millis();
                if (servoState == 1) { // Was pausing at top
                    servoState = 2; // Switch to sweeping down
                } else { // Was pausing at bottom
                    servoState = 0; // Switch to sweeping up
                }
            }
            break;
        }

        case DESCENT: // <-- SECONDARY PARACHUTE DEPLOYMENT STATE
            // Move servo to 90 degrees to deploy the parachute.
            // This check ensures the command is only sent if the servo isn't already there.
            if (myServo.read() != 90) {
                myServo.write(90);
                logEvent("ACTUATOR: Secondary chute servo deployed to 90 deg.");
            }
            break;

        case RECOVERY:
            // During recovery, hold the servo in its deployed position.
            if (myServo.read() != 90) {
                myServo.write(90);
            }
            // You can also add buzzer logic here.
            // For example: if (millis() % 2000 > 1000) { digitalWrite(BUZZER_PIN, HIGH); } else { digitalWrite(BUZZER_PIN, LOW); }
            break;
            
        default:
             // Default to a safe state if the flight state is unknown.
            if (myServo.read() != 0) {
                myServo.write(0);
            }
            break;
    }
}

// ==============================================================================
// COMMUNICATION FUNCTIONS
// ==============================================================================
void sendTelemetry() {
    telemetryData.packetCount++;
    char telemetryString[300]; 
    
    // Updated packet to include BNO08x quaternion data instead of old stubs
    snprintf(telemetryString, sizeof(telemetryString),
             "%s,%.2f,%lu,%d,%.2f,%.0f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\r\n",
             TEAM_ID, 
             telemetryData.timeStamp, 
             telemetryData.packetCount,
             telemetryData.flightState,
             telemetryData.altitude,
             telemetryData.pressure, 
             telemetryData.temperature, 
             telemetryData.voltage,
             telemetryData.gnssLatitude, 
             telemetryData.gnssLongitude, 
             telemetryData.bno_qi, // IMU Data
             telemetryData.bno_qj, // IMU Data
             telemetryData.bno_qk, // IMU Data
             telemetryData.bno_qr, // IMU Data
             telemetryData.gnssSatellites);

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