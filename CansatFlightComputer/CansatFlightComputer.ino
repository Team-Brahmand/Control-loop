// ==============================================================================
// CansatFlightComputer_Enhanced.ino
// Enhanced version with complete state machine and improved reliability
// ==============================================================================

#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include "esp_system.h"
#include "config.h"
#include <Adafruit_Sensor.h>
#include <bsec.h>
#include <ESP32Servo.h>
#include <Adafruit_LPS2X.h>
#include "Adafruit_SHT4x.h"
#include "ICM_20948.h"
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==============================================================================
// HARDWARE CONFIGURATION
// ==============================================================================
#define LPS22_ADDR      0x5D
#define BME_ADDR        0x77
#define ICM_ADDR        0x69
#define OLED_ADDR       0x3C
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64

// ==============================================================================
// STATE MACHINE CONFIGURATION
// ==============================================================================
#define LAUNCH_DETECT_ACCEL_G       3.0f    // Acceleration threshold for launch detection
#define APOGEE_ALTITUDE_DROP_M      10.0f   // Altitude drop to detect apogee
#define DEPLOY_ALTITUDE_M           500.0f  // Altitude for aerobrake release
#define DEPLOY_TOLERANCE_M          10.0f   // Tolerance for deployment altitude
#define LANDING_SPEED_THRESHOLD     1.0f    // m/s - speed below which we consider landed
#define LANDING_ALTITUDE_M          5.0f    // meters above ground to consider landed
#define MIN_ASCENT_ALTITUDE_M       50.0f   // Minimum altitude to consider in ascent

// ==============================================================================
// SENSOR & ACTUATOR OBJECTS
// ==============================================================================
Adafruit_LPS22 lps = Adafruit_LPS22();
Adafruit_Sensor *lps_pressure = NULL;
Adafruit_Sensor *lps_temp = NULL;
TinyGPSPlus gps;
Adafruit_SHT4x sht41 = Adafruit_SHT4x();
Bsec bme680;
ICM_20948_I2C myICM;
Servo myServo;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==============================================================================
// FLIGHT STATE ENUM
// ==============================================================================
enum FlightState {
    BOOT = 0,
    TEST_MODE = 1,
    LAUNCH_PAD = 2,
    ASCENT = 3,
    ROCKET_DEPLOY = 4,
    DESCENT = 5,
    AEROBREAK_RELEASE = 6,
    IMPACT = 7
};

// ==============================================================================
// GLOBAL STATE VARIABLES
// ==============================================================================
volatile bool isInTestMode = false;
bool sdCardOK = false;

// Sensor status flags
bool lps_ok = false, sht_ok = false, bme_ok = false, icm_ok = false;
bool servo_ok = false, lora_ok = false, oled_ok = false;

volatile FlightState currentFlightState = BOOT;
FlightState previousFlightState = BOOT;

// State machine tracking variables
float maxAltitude = 0.0f;
float groundAltitude = 0.0f;
bool altitudeCalibrated = false;
unsigned long stateEntryTime = 0;
unsigned long launchDetectTime = 0;
bool aerobrakeDeployed = false;
bool primaryParachuteDeployed = false;

// Moving average for altitude stability check
#define ALT_BUFFER_SIZE 10
float altitudeBuffer[ALT_BUFFER_SIZE] = {0};
int altBufferIndex = 0;

// ==============================================================================
// TELEMETRY DATA STRUCTURE
// ==============================================================================
volatile struct TelemetryData {
    unsigned long packetCount = 0;
    float timeStamp = 0.0;
    FlightState flightState = BOOT;
    float altitude = NAN;
    float pressure = NAN;
    float temperature = NAN;
    float voltage = NAN;
    float gnssLatitude = 0.0, gnssLongitude = 0.0, gnssAltitude = 0.0;
    int gnssSatellites = 0;
    float imu_ax = NAN, imu_ay = NAN, imu_az = NAN;
    float imu_gx = NAN, imu_gy = NAN, imu_gz = NAN;
    float imu_mx = NAN, imu_my = NAN, imu_mz = NAN;
    float sht41_temperature = NAN;
    float sht41_humidity = NAN;
    float bme680_pressure = NAN;
    float bme680_temperature = NAN;
    float bme680_humidity = NAN;
    float bme680_gas_resistance = NAN;
    float bme680_iaq = NAN;
    float verticalSpeed = 0.0f; // Derived from altitude
} telemetryData;

TaskHandle_t Task_Core0, Task_Core1;
File dataFile, eventFile;
String dataFileName = "";
String eventFileName = "";
SemaphoreHandle_t spiMutex;

// ==============================================================================
// FUNCTION PROTOTYPES
// ==============================================================================
void scanI2CBus();
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
void checkIaqSensorStatus(void);
void updateFlightState();
void manageActuators();
void sendLoRaTelemetry();
void receiveLoRaCommands();
void processCommand(String command);
void displayTelemetry();
String getNextFileName(String baseName, String extension);
void calibrateGroundAltitude();
float calculateVerticalSpeed();
void updateAltitudeBuffer(float alt);
bool isAltitudeStable();
void transitionToState(FlightState newState);
const char* getStateName(FlightState state);

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
    DEBUG_SERIAL.begin(115200);
    delay(2000);

    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, LOW);

    DEBUG_SERIAL.println("======================================");
    DEBUG_SERIAL.printf("CANSAT %s BOOTING...\n", TEAM_ID);
    
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    spiMutex = xSemaphoreCreateMutex();
    if (spiMutex == NULL) {
        DEBUG_SERIAL.println("CRITICAL: Failed to create SPI mutex!");
    }

    scanI2CBus();

    GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

    if (!SD.begin(SD_CS_PIN)) {
        sdCardOK = false;
        logEvent("CRITICAL: SD Card initialization failed!");
    } else {
        sdCardOK = true;
        
        dataFileName = getNextFileName("data_", ".csv");
        eventFileName = getNextFileName("events_", ".csv");
        
        dataFile = SD.open(dataFileName, FILE_WRITE);
        if (dataFile) {
            dataFile.println("TeamID,Timestamp,Packet,State,Altitude,Pressure,Temp,Voltage,Lat,Lon,Sats,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,SHT_Temp,SHT_Hum,BME_Press,BME_Temp,BME_Hum,BME_Gas,BME_IAQ,VerticalSpeed");
            dataFile.close();
        }
        
        eventFile = SD.open(eventFileName, FILE_WRITE);
        if (eventFile) {
            eventFile.println("Timestamp,Event");
            eventFile.close();
        }
        
        logEvent("SD Card Initialized. Data: " + dataFileName + ", Events: " + eventFileName);
    }
    
    setupSensors();
    
    logEvent("Boot complete. Creating tasks.");
    transitionToState(LAUNCH_PAD);
    
    xTaskCreatePinnedToCore(core0_Task, "Core0_Sensors", 10000, NULL, 2, &Task_Core0, 0);
    xTaskCreatePinnedToCore(core1_Task, "Core1_Comms", 10000, NULL, 1, &Task_Core1, 1);

    digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ==============================================================================
// CORE TASKS
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
        displayTelemetry();
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void core1_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CORE_1_LOOP_RATE_HZ);
    for (;;) {
        sendLoRaTelemetry();
        receiveLoRaCommands();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================================================
// STATE TRANSITION MANAGEMENT
// ==============================================================================
void transitionToState(FlightState newState) {
    if (newState != currentFlightState) {
        previousFlightState = currentFlightState;
        currentFlightState = newState;
        stateEntryTime = millis();
        
        String stateMsg = "State transition: " + String(getStateName(previousFlightState)) + 
                         " -> " + String(getStateName(newState));
        logEvent(stateMsg);
        
        // Buzzer feedback for critical state changes
        if (newState == ASCENT || newState == ROCKET_DEPLOY || newState == IMPACT) {
            for (int i = 0; i < 3; i++) {
                digitalWrite(BUZZER_PIN, HIGH);
                delay(100);
                digitalWrite(BUZZER_PIN, LOW);
                delay(100);
            }
        }
    }
}

const char* getStateName(FlightState state) {
    switch (state) {
        case BOOT: return "BOOT";
        case TEST_MODE: return "TEST_MODE";
        case LAUNCH_PAD: return "LAUNCH_PAD";
        case ASCENT: return "ASCENT";
        case ROCKET_DEPLOY: return "ROCKET_DEPLOY";
        case DESCENT: return "DESCENT";
        case AEROBREAK_RELEASE: return "AEROBREAK_RELEASE";
        case IMPACT: return "IMPACT";
        default: return "UNKNOWN";
    }
}

// ==============================================================================
// ALTITUDE TRACKING FUNCTIONS
// ==============================================================================
void calibrateGroundAltitude() {
    if (!lps_ok || isnan(telemetryData.altitude)) return;
    
    groundAltitude = telemetryData.altitude;
    altitudeCalibrated = true;
    logEvent("Ground altitude calibrated to: " + String(groundAltitude, 2) + " m");
}

void updateAltitudeBuffer(float alt) {
    altitudeBuffer[altBufferIndex] = alt;
    altBufferIndex = (altBufferIndex + 1) % ALT_BUFFER_SIZE;
}

bool isAltitudeStable() {
    float sum = 0;
    float mean = 0;
    float variance = 0;
    
    for (int i = 0; i < ALT_BUFFER_SIZE; i++) {
        sum += altitudeBuffer[i];
    }
    mean = sum / ALT_BUFFER_SIZE;
    
    for (int i = 0; i < ALT_BUFFER_SIZE; i++) {
        variance += pow(altitudeBuffer[i] - mean, 2);
    }
    variance /= ALT_BUFFER_SIZE;
    
    return (sqrt(variance) < 2.0f); // Less than 2m standard deviation
}

float calculateVerticalSpeed() {
    static float lastAltitude = 0.0f;
    static unsigned long lastTime = 0;
    
    unsigned long currentTime = millis();
    float currentAlt = telemetryData.altitude;
    
    if (lastTime == 0 || isnan(currentAlt)) {
        lastTime = currentTime;
        lastAltitude = currentAlt;
        return 0.0f;
    }
    
    float deltaTime = (currentTime - lastTime) / 1000.0f; // Convert to seconds
    if (deltaTime < 0.1f) return telemetryData.verticalSpeed; // Use previous value
    
    float speed = (currentAlt - lastAltitude) / deltaTime;
    lastAltitude = currentAlt;
    lastTime = currentTime;
    
    return speed;
}

// ==============================================================================
// ENHANCED STATE MACHINE
// ==============================================================================
void updateFlightState() {
    telemetryData.flightState = currentFlightState;
    
    // Update vertical speed
    telemetryData.verticalSpeed = calculateVerticalSpeed();
    
    // Update altitude buffer for stability checks
    if (!isnan(telemetryData.altitude)) {
        updateAltitudeBuffer(telemetryData.altitude);
    }
    
    // Get relative altitude
    float relativeAlt = altitudeCalibrated ? 
                       (telemetryData.altitude - groundAltitude) : telemetryData.altitude;
    
    // Compute acceleration magnitude
    float accelMagnitude = sqrt(pow(telemetryData.imu_ax, 2) + 
                               pow(telemetryData.imu_ay, 2) + 
                               pow(telemetryData.imu_az, 2));
    
    // STATE MACHINE LOGIC
    switch (currentFlightState) {
        
        case BOOT:
            // Should transition to LAUNCH_PAD after setup
            break;
            
        case TEST_MODE:
            // In test mode, stay here unless commanded otherwise
            if (!isInTestMode) {
                transitionToState(LAUNCH_PAD);
            }
            // Send telemetry and wait for commands
            
            break;
            
        case LAUNCH_PAD:
            // Calibrate ground altitude if not done
            if (!altitudeCalibrated && lps_ok) {
                calibrateGroundAltitude();
            }
            
            // Detect launch by high acceleration
            if (accelMagnitude > LAUNCH_DETECT_ACCEL_G) {
                launchDetectTime = millis();
                transitionToState(ASCENT);
                primaryParachuteDeployed = true; // Deployed at ejection
            }
            break;
            
        case ASCENT:
            // Track maximum altitude
            if (relativeAlt > maxAltitude) {
                maxAltitude = relativeAlt;
            }
            
            // Detect apogee (altitude starts dropping)
            if (maxAltitude > MIN_ASCENT_ALTITUDE_M && 
                relativeAlt < (maxAltitude - APOGEE_ALTITUDE_DROP_M)) {
                transitionToState(ROCKET_DEPLOY);
            }
            
            // Safety timeout - if in ascent for more than 2 minutes
            if ((millis() - stateEntryTime) > 120000) {
                logEvent("ASCENT timeout - forcing ROCKET_DEPLOY");
                transitionToState(ROCKET_DEPLOY);
            }
            break;
            
        case ROCKET_DEPLOY:
            // This state represents apogee and separation
            // Transition immediately to descent
            transitionToState(DESCENT);
            break;
            
        case DESCENT:
            // Check if we've reached deployment altitude
            if (relativeAlt <= (DEPLOY_ALTITUDE_M + DEPLOY_TOLERANCE_M) &&
                !aerobrakeDeployed) {
                transitionToState(AEROBREAK_RELEASE);
            }
            
            // Check for landing
            if (relativeAlt < LANDING_ALTITUDE_M && 
                abs(telemetryData.verticalSpeed) < LANDING_SPEED_THRESHOLD) {
                transitionToState(IMPACT);
            }
            
            // Safety check - if altitude is very low
            if (relativeAlt < 10.0f && (millis() - stateEntryTime) > 5000) {
                transitionToState(IMPACT);
            }
            break;
            
        case AEROBREAK_RELEASE:
            // Stay in this state briefly, then return to descent
            if ((millis() - stateEntryTime) > 2000) {
                transitionToState(DESCENT);
            }
            
            // Also check for landing
            if (relativeAlt < LANDING_ALTITUDE_M && 
                abs(telemetryData.verticalSpeed) < LANDING_SPEED_THRESHOLD) {
                transitionToState(IMPACT);
            }
            break;
            
        case IMPACT:
            // Terminal state - activate beacon
            // Could implement automatic mode reset after extended period
            if ((millis() - stateEntryTime) > 300000) { // 5 minutes
                logEvent("Extended ground time - system stabilized");
            }
            break;
    }
}

// ==============================================================================
// ACTUATOR MANAGEMENT
// ==============================================================================
void manageActuators() {
    switch (currentFlightState) {
        case BOOT:
        case TEST_MODE:
        case LAUNCH_PAD:
            // Servo at neutral position
            if (servo_ok && myServo.read() != 0) {
                myServo.write(0);
            }
            // Buzzer off
            digitalWrite(BUZZER_PIN, LOW);
            break;
            
        case ASCENT:
            // Keep servo at neutral during ascent
            if (servo_ok && myServo.read() != 0) {
                myServo.write(0);
            }
            break;
            
        case ROCKET_DEPLOY:
            // No actuator action needed at apogee
            break;
            
        case DESCENT:
            // Primary parachute is already deployed
            // Keep servo at neutral until aerobrake release
            if (servo_ok && myServo.read() != 0) {
                myServo.write(0);
            }
            break;
            
        case AEROBREAK_RELEASE:
            // Deploy aerobrake/secondary parachute
            if (!aerobrakeDeployed && servo_ok) {
                myServo.write(90); // Deploy position
                aerobrakeDeployed = true;
                logEvent("AEROBRAKE DEPLOYED at altitude: " + 
                        String(telemetryData.altitude - groundAltitude, 2) + " m");
                
                // Buzzer confirmation
                for (int i = 0; i < 5; i++) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(50);
                    digitalWrite(BUZZER_PIN, LOW);
                    delay(50);
                }
            }
            break;
            
        case IMPACT:
            // Activate audio beacon continuously
            static unsigned long lastBeepTime = 0;
            if (millis() - lastBeepTime > 1000) {
                digitalWrite(BUZZER_PIN, HIGH);
                delay(200);
                digitalWrite(BUZZER_PIN, LOW);
                lastBeepTime = millis();
            }
            
            // Return servo to neutral
            if (servo_ok && myServo.read() != 90) {
                myServo.write(0);
            }
            break;
    }
}

// ==============================================================================
// I2C BUS SCANNER
// ==============================================================================
void scanI2CBus() {
    DEBUG_SERIAL.println("Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            DEBUG_SERIAL.print(" - I2C device found at address 0x");
            if (address < 16) DEBUG_SERIAL.print("0");
            DEBUG_SERIAL.println(address, HEX);
            nDevices++;
        }
    }
    if (nDevices == 0) {
        logEvent("CRITICAL: No I2C devices found! Check wiring.");
    } else {
        DEBUG_SERIAL.println("-> Scan complete.");
    }
}

// ==============================================================================
// DATA LOGGING
// ==============================================================================
void logEvent(String eventMessage) {
    String logEntryCSV = String(millis() / 1000.0, 2) + "," + eventMessage;
    String logEntryHuman = "EVENT: " + String(millis() / 1000.0, 2) + "s: " + eventMessage;
    DEBUG_SERIAL.println(logEntryHuman);
    if (sdCardOK && eventFileName.length() > 0) {
        if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            eventFile = SD.open(eventFileName, FILE_APPEND);
            if (eventFile) {
                eventFile.println(logEntryCSV);
                eventFile.close();
            }
            xSemaphoreGive(spiMutex);
        }
    }
}

void logDataToSD() {
    if (!sdCardOK || dataFileName.length() == 0) return;
    
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        dataFile = SD.open(dataFileName, FILE_APPEND);
        if (dataFile) {
            char dataString[500];
            snprintf(dataString, sizeof(dataString), 
                "%s,%.2f,%lu,%d,%.2f,%.0f,%.2f,%.2f,%.6f,%.6f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.2f,%.2f,%.0f,%.1f,%.2f",
                TEAM_ID, telemetryData.timeStamp, telemetryData.packetCount, 
                telemetryData.flightState, telemetryData.altitude, 
                telemetryData.pressure, telemetryData.temperature, 
                telemetryData.voltage, telemetryData.gnssLatitude, 
                telemetryData.gnssLongitude, telemetryData.gnssSatellites,
                telemetryData.imu_ax, telemetryData.imu_ay, telemetryData.imu_az,
                telemetryData.imu_gx, telemetryData.imu_gy, telemetryData.imu_gz,
                telemetryData.imu_mx, telemetryData.imu_my, telemetryData.imu_mz,
                telemetryData.sht41_temperature, telemetryData.sht41_humidity,
                telemetryData.bme680_pressure, telemetryData.bme680_temperature, 
                telemetryData.bme680_humidity, telemetryData.bme680_gas_resistance, 
                telemetryData.bme680_iaq, telemetryData.verticalSpeed);
            dataFile.println(dataString);
            dataFile.close();
        }
        xSemaphoreGive(spiMutex);
    }
}

// ==============================================================================
// SENSOR SETUP
// ==============================================================================
void setupSensors() {
    logEvent("Initializing detected sensors...");

    Wire.beginTransmission(OLED_ADDR);
    if (Wire.endTransmission() == 0) {
        DEBUG_SERIAL.print("Initializing OLED... ");
        if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("OLED OK");
            display.display();
            oled_ok = true;
            DEBUG_SERIAL.println("OK");
        } else {
            DEBUG_SERIAL.println("FAILED!");
        }
    } else {
        logEvent("OLED not found on I2C bus.");
    }

    Wire.beginTransmission(LPS22_ADDR);
    if (Wire.endTransmission() == 0) {
        DEBUG_SERIAL.print("Initializing LPS22... ");
        if (lps.begin_I2C(LPS22_ADDR)) {
            lps_pressure = lps.getPressureSensor();
            lps_temp = lps.getTemperatureSensor();
            lps_ok = true;
            DEBUG_SERIAL.println("OK");
        } else {
            DEBUG_SERIAL.println("FAILED!");
        }
    } else {
        logEvent("LPS22 not found on I2C bus.");
    }
    
    Wire.beginTransmission(ICM_ADDR);
    if (Wire.endTransmission() == 0) {
        DEBUG_SERIAL.print("Initializing ICM-20948... ");
        myICM.begin(Wire, ICM_ADDR);
        if (myICM.status == ICM_20948_Stat_Ok) {
            icm_ok = true;
            DEBUG_SERIAL.println("OK");
        } else {
            DEBUG_SERIAL.println("FAILED!");
        }
    } else {
        logEvent("ICM-20948 not found on I2C bus.");
    }

    DEBUG_SERIAL.print("Initializing SHT41... ");
    if (sht41.begin()) {
        sht_ok = true;
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
        logEvent("SHT41 not found or failed to init.");
    }
    
    Wire.beginTransmission(BME_ADDR);
    if (Wire.endTransmission() == 0) {
        DEBUG_SERIAL.print("Initializing BME680... ");
        bme680.begin(BME_ADDR, Wire);
        if (bme680.bsecStatus == BSEC_OK && bme680.bme68xStatus == BME68X_OK) {
            bsec_virtual_sensor_t sensorList[] = {
                BSEC_OUTPUT_RAW_TEMPERATURE, BSEC_OUTPUT_RAW_PRESSURE, 
                BSEC_OUTPUT_RAW_HUMIDITY, BSEC_OUTPUT_RAW_GAS, BSEC_OUTPUT_IAQ
            };
            bme680.updateSubscription(sensorList, 5, BSEC_SAMPLE_RATE_LP);
            bme_ok = true;
            DEBUG_SERIAL.println("OK");
        } else {
            DEBUG_SERIAL.println("FAILED!");
            checkIaqSensorStatus();
        }
    } else {
        logEvent("BME680 not found on I2C bus.");
    }

    DEBUG_SERIAL.print("Initializing Servo... ");
    ESP32PWM::allocateTimer(0);
    if (myServo.attach(SERVO_PIN)) {
        servo_ok = true;
        myServo.write(0); // Start at neutral
        DEBUG_SERIAL.println("OK");
    } else {
        DEBUG_SERIAL.println("FAILED!");
    }

    DEBUG_SERIAL.print("Initializing LoRa SX1278... ");
    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (!LoRa.begin(433E6)) {
        DEBUG_SERIAL.println("FAILED!");
    } else {
        LoRa.setSyncWord(0xF3);
        lora_ok = true;
        DEBUG_SERIAL.println("OK");
    }
    
    logEvent("Sensor init check complete.");
}

// ==============================================================================
// SENSOR READING
// ==============================================================================
void readAllSensors() {
    telemetryData.timeStamp = millis() / 1000.0f;
    
    if (lps_ok) readBarometer();
    if (icm_ok) readIMU();
    if (sht_ok) readSHT41();
    if (bme_ok) readBME680();
    
    readGPS();
    readVoltage();
}

void readBarometer() {
    sensors_event_t pressure_event, temp_event;
    if (lps_pressure->getEvent(&pressure_event) && lps_temp->getEvent(&temp_event)) {
        telemetryData.pressure = pressure_event.pressure * 100;
        telemetryData.temperature = temp_event.temperature;
        if (telemetryData.pressure > 0) {
            telemetryData.altitude = 44330 * (1.0f - pow(telemetryData.pressure / 
                                       (SEALEVELPRESSURE_HPA * 100), 0.1903));
        }
    } else {
        logEvent("Failed to read LPS22.");
        lps_ok = false;
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

void readIMU() {
    if (myICM.dataReady()) {
        myICM.getAGMT();
        telemetryData.imu_ax = myICM.accX();
        telemetryData.imu_ay = myICM.accY();
        telemetryData.imu_az = myICM.accZ();
        telemetryData.imu_gx = myICM.gyrX();
        telemetryData.imu_gy = myICM.gyrY();
        telemetryData.imu_gz = myICM.gyrZ();
        telemetryData.imu_mx = myICM.magX();
        telemetryData.imu_my = myICM.magY();
        telemetryData.imu_mz = myICM.magZ();
    }
}

void readVoltage() {
    int rawADC = analogRead(VOLTAGE_PIN);
    telemetryData.voltage = (rawADC / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
}

void readSHT41() {
    sensors_event_t humidity, temp;
    if (sht41.getEvent(&humidity, &temp)) {
        telemetryData.sht41_temperature = temp.temperature;
        telemetryData.sht41_humidity = humidity.relative_humidity;
    } else {
        logEvent("Failed to read SHT41.");
        sht_ok = false;
    }
}

void readBME680() {
    if (bme680.run()) {
        telemetryData.bme680_pressure = bme680.pressure;
        telemetryData.bme680_temperature = bme680.temperature;
        telemetryData.bme680_humidity = bme680.humidity;
        telemetryData.bme680_gas_resistance = bme680.gasResistance;
        telemetryData.bme680_iaq = bme680.iaq;
    } else {
        checkIaqSensorStatus();
    }
}

void checkIaqSensorStatus(void) {
    if (bme680.bsecStatus != BSEC_OK) {
        logEvent("BSEC Error/Warning: " + String(bme680.bsecStatus));
    }
    if (bme680.bme68xStatus != BME68X_OK) {
        logEvent("BME680 Error/Warning: " + String(bme680.bme68xStatus));
    }
}

// ==============================================================================
// OLED DISPLAY
// ==============================================================================
void displayTelemetry() {
    if (!oled_ok) return;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    
    float relAlt = altitudeCalibrated ? 
                  (telemetryData.altitude - groundAltitude) : telemetryData.altitude;
    
    display.printf("S:%s\n", getStateName(currentFlightState));
    display.printf("Alt:%.1fm (%.1f)\n", telemetryData.altitude, relAlt);
    display.printf("VS:%.1fm/s Max:%.1f\n", telemetryData.verticalSpeed, maxAltitude);
    display.printf("P:%.0fPa T:%.1fC\n", telemetryData.pressure, telemetryData.temperature);
    display.printf("Acc:%.1fg\n", 
                   sqrt(pow(telemetryData.imu_ax, 2) + 
                        pow(telemetryData.imu_ay, 2) + 
                        pow(telemetryData.imu_az, 2)));
    display.printf("GPS:%d Pkt:%lu\n", telemetryData.gnssSatellites, 
                   telemetryData.packetCount);
    display.printf("V:%.2fV T:%.0fs", telemetryData.voltage, telemetryData.timeStamp);
    display.display();
}

// ==============================================================================
// COMMUNICATION FUNCTIONS
// ==============================================================================
void sendLoRaTelemetry() {
    if (!lora_ok) return;
    
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        telemetryData.packetCount++;
        char telemetryString[350];
        
        float relAlt = altitudeCalibrated ? 
                      (telemetryData.altitude - groundAltitude) : telemetryData.altitude;
        
        snprintf(telemetryString, sizeof(telemetryString),
                 "%s,%.2f,%lu,%d,%.2f,%.0f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%d,%.2f",
                 TEAM_ID, 
                 telemetryData.timeStamp, 
                 telemetryData.packetCount,
                 telemetryData.flightState,
                 relAlt,
                 telemetryData.pressure, 
                 telemetryData.temperature, 
                 telemetryData.voltage,
                 telemetryData.gnssLatitude, 
                 telemetryData.gnssLongitude, 
                 telemetryData.imu_ax,
                 telemetryData.imu_gy,
                 telemetryData.gnssSatellites,
                 telemetryData.verticalSpeed);

        LoRa.beginPacket();
        LoRa.print(telemetryString);
        LoRa.endPacket();
        
        xSemaphoreGive(spiMutex);
        
        if (DEBUG_PRINT_ENABLED) {
            DEBUG_SERIAL.print("LORA_TX-> " + String(telemetryString) + "\n");
        }
    }
}

void receiveLoRaCommands() {
    if (!lora_ok) return;
    
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        unsigned long listenStart = millis();
        const unsigned long LISTEN_WINDOW_MS = 200;
        
        while (millis() - listenStart < LISTEN_WINDOW_MS) {
            int packetSize = LoRa.parsePacket();
            if (packetSize) {
                String receivedCommand = "";
                while (LoRa.available()) {
                    receivedCommand += (char)LoRa.read();
                }
                
                xSemaphoreGive(spiMutex);
                
                if (DEBUG_PRINT_ENABLED) {
                    DEBUG_SERIAL.println(" | LORA_RX<- " + receivedCommand);
                }
                processCommand(receivedCommand);
                return;
            }
            delay(10);
        }
        
        xSemaphoreGive(spiMutex);
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
            transitionToState(TEST_MODE);
        }
    } 
    else if (command.equalsIgnoreCase("CMD,SET_MODE,FLIGHT")) {
        if (isInTestMode) {
            logEvent("SWITCHING TO FLIGHT MODE.");
            isInTestMode = false;
            transitionToState(LAUNCH_PAD);
        }
    }
    else if (command.equalsIgnoreCase("CMD,CALIBRATE")) {
        calibrateGroundAltitude();
        logEvent("Manual calibration requested");
    }
    else if (command.equalsIgnoreCase("CMD,RESET_STATE")) {
        maxAltitude = 0.0f;
        aerobrakeDeployed = false;
        primaryParachuteDeployed = false;
        transitionToState(LAUNCH_PAD);
        logEvent("State machine reset to LAUNCH_PAD");
    }
    else if (command.equalsIgnoreCase("CMD,DEPLOY_TEST")) {
        if (currentFlightState == TEST_MODE || currentFlightState == LAUNCH_PAD) {
            if (servo_ok) {
                myServo.write(90);
                delay(1000);
                myServo.write(0);
                logEvent("Manual servo deployment test executed");
            }
        }
    }
    else if (command.equalsIgnoreCase("CMD,STATUS")) {
        String status = "State:" + String(getStateName(currentFlightState)) + 
                       " Alt:" + String(telemetryData.altitude, 1) + 
                       " MaxAlt:" + String(maxAltitude, 1) +
                       " Deploy:" + String(aerobrakeDeployed ? "Y" : "N");
        logEvent(status);
    }
    else {
        logEvent("Unknown command: " + command);
    }
}

// ==============================================================================
// FILE NAMING FUNCTIONS
// ==============================================================================
String getNextFileName(String baseName, String extension) {
    int fileNumber = 1;
    String fileName = "/" + baseName + String(fileNumber) + extension;
    
    while (SD.exists(fileName)) {
        fileNumber++;
        fileName = "/" + baseName + String(fileNumber) + extension;
        
        if (fileNumber > 9999) {
            fileName = "/" + baseName + "9999" + extension;
            break;
        }
    }
    
    DEBUG_SERIAL.println("Next available file: " + fileName);
    return fileName;
