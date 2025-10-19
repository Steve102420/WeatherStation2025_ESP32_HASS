#include <Arduino.h>                           // Arduino core library
#include <Ticker.h>                            // Timer library
#include "driver/pcnt.h"                       // Pulse counter library
#include <Wire.h>                              // I2C library
#include <WiFi.h>                              // WiFi library
#include <PubSubClient.h>                      // MQTT library
#include <ArduinoJson.h>                       // JSON library
#include <Adafruit_BME280.h>                   // BME280 sensor 0x76
#include <Credentials.h>                       // Credentials
#include <SparkFun_VEML6075_Arduino_Library.h> // UV Sensor 0x10
#include "SparkFun_AS3935.h"                   // Lightning sensor 0x03
#include <BH1750.h>                            // Light sensor 0x23
#include "AS5600.h"                            // Magnetic angle sensor 0x36
#include <QMC5883LCompass.h>                   // Compass sensor 0x0D
//----------------------------------------------------------------------------------------
// Define constants
#define PERIOD_MILLSEC_1000         1000
#define PERIOD_MILLSEC_500          500
#define PERIOD_MILLSEC_250          250

#define AS3935_ADDR                 0x03
#define INDOOR                      0x12
#define OUTDOOR                     0xE
#define LIGHTNING_INT               0x08
#define DISTURBER_INT               0x04
#define NOISE_INT                   0x01

#define WIND_SPEED_PIN              23
#define WIND_DIRECTION_PIN          19
#define RAIN_GAUGE_PIN              32
#define LIGHTNING_INT_PIN           18

#define BATT_VOLTAGE_MEAS_PIN       36
#define BATT_VOLTAGE_MEAS_SWITCH    13

//PCNT definitions
#define WIND_PCNT_UNIT              PCNT_UNIT_0
#define RAIN_PCNT_UNIT              PCNT_UNIT_1
#define SAMPLE_INTERVAL_MS          2000                // Mintavételezés 2 másodpercenként
#define AVERAGING_PERIOD_MS         600000              // 10 perc = 600000 ms

#define RADIUS_M                    0.10f               // Forgás középpont – félgömb közepe távolság (m)
#define IMPULSES_REV                1                   // Impulzus / fordulat
#define SECONDS_IN_HOUR             3600.0f
#define FRICTION_COMPENSATION       1.5f

//----------------------------------------------------------------------------------------
const char *ssid = WIFI_SSID;                   // WiFi SSID
const char *password = WIFI_PASSWORD;           // WiFi Password
const char *mqtt_server = MQTT_SERVER_IP;       // MQTT Server IP, same of Home Assistant
const char *mqttUser = MQTT_SERVER_USERNAME;    // MQTT Server User Name
const char *mqttPsw = MQTT_SERVER_PASSWORD;     // MQTT Server password
int mqttPort = MQTT_SERVER_PORT;                // MQTT Server Port

// MQTT Discovery
const char *deviceModel = "ESP32Device";                 // Hardware Model
const char *swVersion = "1.0";                           // Firmware Version
const char *manufacturer = "Vigasan";                    // Manufacturer Name
String deviceName = "CustomSensor";                      // Device Name
String mqttStatusTopic = "esp32iotsensor/" + deviceName; // MQTT Topic

// Global variables
WiFiClient WiFiClient;
PubSubClient mqttPubSub(WiFiClient);
//unsigned long Time = 0;
//int count = 0;

int mqttCounterConn = 0;
String UniqueId;

// Wind sensor variables
int16_t windCount = 0;
int16_t rainCount = 0;
int16_t windDirection = 0;
Ticker sampleTimer;
Ticker reportTimer;

// Új: periódikus ticker a teljes mérés/publish időzítésére
Ticker periodicTimer;
volatile bool measurementDue = false;

volatile int32_t totalPulses = 0;
volatile int16_t lastSamplePulses = 0;
volatile float maxSpeed = 0.0f;
volatile float avgSpeed = 0.0f;
volatile uint32_t sampleCount = 0;


// BME280 Sensor
Adafruit_BME280 bme;
float temperature, humidity, pressure;

// Light sensor
BH1750 lightMeter(0x23);
float lux = 0;

// Lightning sensor
SparkFun_AS3935 lightning(AS3935_ADDR);
const int lightningInt = LIGHTNING_INT_PIN;
byte noiseFloor = 1;
byte watchDogVal = 1;
byte spike = 1;
byte lightningThresh = 1;
char message[] = "";
byte intVal = 0;
byte distance = 0;
byte noise_flag = 0;
byte disturber_flag = 0;
long lightEnergy = 0;
volatile bool lightningDetected = false;

// UV sensor
VEML6075 uv;
uint16_t rawA, rawB, visibleComp, irComp;
float uviaCalc, uvibCalc, uvia, uvib, uvi;
const float CALIBRATION_ALPHA_VIS = 1.0;
const float CALIBRATION_BETA_VIS = 1.0;
const float CALIBRATION_GAMMA_IR = 1.0;
const float CALIBRATION_DELTA_IR = 1.0;
const float UVA_RESPONSIVITY = 0.00110;
const float UVB_RESPONSIVITY = 0.00125;
const float UVA_VIS_COEF_A = 2.22;
const float UVA_IR_COEF_B = 1.33;
const float UVB_VIS_COEF_C = 2.95;
const float UVB_IR_COEF_D = 1.75;

//AS5600 Magnetic angle sensor
AS5600 as5600;

//QMC5883L Compass sensor
QMC5883LCompass compass;
uint8_t azimuth = 0;
uint8_t sensor_orientation_offset = 0;

//Battery voltage measurement
uint16_t batteryVoltage_RawADC = 0;
uint16_t batteryVoltage = 0;
//----------------------------------------------------------------------------------------

// Function declarations
void setup_wifi(void);
void mqttReconnect(void);
void mqttReceiverCallback(char *topic, byte *inFrame, unsigned int length);
void initSystem(void);
void initSensors(void);
void readVEML6075(void);
void readBH1750(void);
void readBME280(void);
void readQMC5883L(void);
void readWindDirection(void);
void printDebugInfo(void);
void ICACHE_RAM_ATTR lightningISR(void);
void setupWindPCNT(void);
void setupRainPCNT(void);
void readCounts(void);
float calculateWindSpeedKmh(int16_t pulses, float intervalSec);
void sampleWind(void);
void reportWind(void);
void batteryVoltageMeasurement(void);
void performPeriodicMeasurement(void);
void IRAM_ATTR periodicTickerCallback(void);
//----------------------------------------------------------------------------------------
void setup()
{
    initSystem();
    initSensors();
    setup_wifi();

    mqttPubSub.setServer(mqtt_server, mqttPort);
    mqttPubSub.setCallback(mqttReceiverCallback);

    // Periodikus mérés: 60 000 ms = 60 s (ugyanaz, mint eredetileg: 6 * 10s)
    // A ticker csak beállít egy flag-et (measurementDue), a tényleges mérés a loop()-ban lesz.
    periodicTimer.attach_ms(AVERAGING_PERIOD_MS, periodicTickerCallback);
}

void loop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!mqttPubSub.connected())
        {
            mqttReconnect();
        }
        else
        {
            mqttPubSub.loop();
        }
    }
    else
    {
        setup_wifi();
    }

    // Ha a ticker jelezte, hogy mérés esedékes - végezd el itt (nem megszakításban)
    if (measurementDue)
    {
        measurementDue = false;
        performPeriodicMeasurement();
    }

    /*if (millis() - Time > 10000)
    {
        Time = millis();

        if (count++ == 5)
        {
            count = 0;

            readVEML6075();
            readBH1750();
            readBME280();
            readQMC5883L();
            readWindDirection();
            batteryVoltageMeasurement();
            //printDebugInfo();

            // JSON payload építése
            StaticJsonDocument<600> doc;
            doc["temperature"] = temperature;
            doc["humidity"] = humidity;
            doc["atmospheric_pressure"] = pressure;
            doc["lux"] = lux;
            doc["uvi"] = uvi;
            doc["wind_speed"] = avgSpeed;
            doc["wind_direction"] = windDirection;
            doc["azimuth"] = azimuth;
            doc["rainfall"] = rainCount;
            
            
            if (distance > 0)
            {
                doc["lightningdistance"] = distance;
            }
            if (lightEnergy > 0)
            {
                doc["lightningenergy"] = lightEnergy;
            }
            if (noise_flag > 0)
            {
                doc["noise"] = noise_flag;
            }
            if (disturber_flag > 0)
            {
                doc["disturber"] = disturber_flag;
            }
            
            char jsonBuffer[600];
            serializeJson(doc, jsonBuffer);

            // Publish
            const char *topic = "office/sensor/demo/state";
            mqttPubSub.publish(topic, jsonBuffer);

            //Serial.print("MQTT: Send Data -> ");
            //Serial.print(topic);
            //Serial.print(": ");
            //Serial.println(jsonBuffer);

            distance = 0;
            lightEnergy = 0;
            noise_flag = 0;
            disturber_flag = 0;
        }
    }*/
    
    if (lightningDetected)
    {
        lightningDetected = false;
        delay(2); // 2ms delay minimum to allow the sensor to process the interrupt
        // Hardware has alerted us to an event, now we read the interrupt register
        // to see exactly what it is.
        intVal = lightning.readInterruptReg();
        if (intVal == NOISE_INT)
        {
            noise_flag = 1;
            Serial.println("Noise.");
        }
        else if (intVal == DISTURBER_INT)
        {
            disturber_flag = 1;
            Serial.println("Disturber.");
        }
        else if (intVal == LIGHTNING_INT)
        {
            Serial.println("Lightning Strike Detected!");
            distance = lightning.distanceToStorm();
            Serial.print("Approximately: ");
            Serial.print(distance);
            Serial.println("km away!");

            lightEnergy = lightning.lightningEnergy();
            Serial.print("Lightning Energy: ");
            Serial.println(lightEnergy);
        }
    }
}
//----------------------------------------------------------------------------------------


void setup_wifi(void)
{
    int counter = 0;
    byte mac[6];
    delay(10);

    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    WiFi.macAddress(mac);
    UniqueId = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);

    Serial.print("Unique ID: ");
    Serial.println(UniqueId);

    while (WiFi.status() != WL_CONNECTED && counter++ < 8)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("");

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("WiFi NOT connected!!!");
    }
}
void mqttReconnect(void)
{
    // Loop until we're reconnected
    while (!mqttPubSub.connected() && (mqttCounterConn++ < 4))
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqttPubSub.connect(deviceName.c_str(), mqttUser, mqttPsw))
        {
            Serial.println("connected");
            // Subscribe
            mqttPubSub.subscribe("homeassistant/status");
            delay(100);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttPubSub.state());
            Serial.println(" try again in 1 seconds");
            delay(1000);
        }
    }
    mqttCounterConn = 0;
}
void mqttReceiverCallback(char *topic, byte *inFrame, unsigned int length)
{
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;

    for (int i = 0; i < length; i++)
    {
        Serial.print((char)inFrame[i]);
        messageTemp += (char)inFrame[i];
    }
    Serial.println();
}
void initSystem(void)
{
    Serial.begin(115200);
    delay(100);
    Serial.println("Weather Station Initializing...");
    Wire.begin(21, 22);
}
void initSensors(void)
{
    // BME280 sensor initialization
    if (!bme.begin(0x76))
    {
        Serial.println("Could not find BME280 sensor!");
    }
    else
    {
        Serial.println("BME280 sensor initialized successfully.");
    }
    

    // Initialize the lightning sensor
    // When lightning is detected the interrupt pin goes HIGH.
    pinMode(lightningInt, INPUT);
    if (!lightning.begin())
    {
        Serial.println("Lightning Detector did not start up, freezing!");
        // while(1);
    }
    else
    {
        Serial.println("Schmow-ZoW, Lightning Detector Ready!\n");
    }
    
    lightning.maskDisturber(false);
    int maskVal = lightning.readMaskDisturber();
    Serial.print("Are disturbers being masked: ");
    if (maskVal == 1)
        Serial.println("YES");
    else if (maskVal == 0)
        Serial.println("NO");

    // The lightning detector defaults to an indoor setting (less
    // gain/sensitivity), if you plan on using this outdoors
    // uncomment the following line:
    lightning.setIndoorOutdoor(OUTDOOR);
    int enviVal = lightning.readIndoorOutdoor();
    Serial.print("Are we set for indoor or outdoor: ");
    if (enviVal == INDOOR)
        Serial.println("Indoor.");
    else if (enviVal == OUTDOOR)
        Serial.println("Outdoor");
    else
        Serial.println(enviVal, BIN);

    // Noise floor setting from 1-7, one being the lowest. Default setting is
    // two. If you need to check the setting, the corresponding function for
    // reading the function follows.
    lightning.setNoiseLevel(noiseFloor);

    int noiseVal = lightning.readNoiseLevel();
    Serial.print("Noise Level is set at: ");
    Serial.println(noiseVal);

    // Watchdog threshold setting can be from 1-10, one being the lowest. Default setting is
    // two. If you need to check the setting, the corresponding function for
    // reading the function follows.
    lightning.watchdogThreshold(watchDogVal);

    int watchVal = lightning.readWatchdogThreshold();
    Serial.print("Watchdog Threshold is set to: ");
    Serial.println(watchVal);

    // Spike Rejection setting from 1-11, one being the lowest. Default setting is
    // two. If you need to check the setting, the corresponding function for
    // reading the function follows.
    // The shape of the spike is analyzed during the chip's
    // validation routine. You can round this spike at the cost of sensitivity to
    // distant events.
    lightning.spikeRejection(spike);

    int spikeVal = lightning.readSpikeRejection();
    Serial.print("Spike Rejection is set to: ");
    Serial.println(spikeVal);

    // This setting will change when the lightning detector issues an interrupt.
    // For example you will only get an interrupt after five lightning strikes
    // instead of one. Default is one, and it takes settings of 1, 5, 9 and 16.
    // Followed by its corresponding read function. Default is zero.

    lightning.lightningThreshold(lightningThresh);

    uint8_t lightVal = lightning.readLightningThreshold();
    Serial.print("The number of strikes before interrupt is triggerd: ");
    Serial.println(lightVal);

    // When the distance to the storm is estimated, it takes into account other
    // lightning that was sensed in the past 15 minutes. If you want to reset
    // time, then you can call this function.

    // lightning.clearStatistics();

    // The power down function has a BIG "gotcha". When you wake up the board
    // after power down, the internal oscillators will be recalibrated. They are
    // recalibrated according to the resonance frequency of the antenna - which
    // should be around 500kHz. It's highly recommended that you calibrate your
    // antenna before using these two functions, or you run the risk of schewing
    // the timing of the chip.

    // lightning.powerDown();
    // delay(1000);
    // if( lightning.wakeUp() )
    //  Serial.println("Successfully woken up!");
    // else
    // Serial.println("Error recalibrating internal osciallator on wake up.");
    // Set too many features? Reset them all with the following function.
    //lightning.resetSettings();
    //delay(100);
    //lightning.calibrateOsc(); // Calibrate the internal oscillator
    //delay(100);

    if(1 == lightning.calibrateOsc())
    {
        delay(100);
        Serial.println("Lightning Detector Oscillator calibrated successfully.");
    }
    else
    {
        Serial.println("Error calibrating Lightning Detector Oscillator.");
    }

    attachInterrupt(digitalPinToInterrupt(lightningInt), lightningISR, RISING);

    // BME280 sensor initialization
    if (!bme.begin(0x76))
    {
        Serial.println("Could not find BME280 sensor!");
    }
    else
    {
        Serial.println("BME280 sensor initialized successfully.");
    }

    // VEML6075 init
    if (uv.begin() == false)
    {
        Serial.println("Unable to communicate with VEML6075.");
    }
    else
    {
        Serial.println("VEML6075 initialised!");
    }
    // Integration time and high-dynamic values will change the UVA/UVB sensitivity. That means
    // new responsivity values will need to be measured for every combination of these settings.
    uv.setIntegrationTime(VEML6075::IT_100MS);
    uv.setHighDynamic(VEML6075::DYNAMIC_NORMAL);

    // BH1750 init
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2))
    {
        Serial.println(F("BH1750 Initialised!"));
    }
    else
    {
        Serial.println(F("Error initialising BH1750!"));
    }

    // Compass sensor init
    compass.init();

    // Wind and rain sensor init
    pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
    pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);
    setupWindPCNT();
    setupRainPCNT();
    //sampleTimer.attach_ms(SAMPLE_INTERVAL_MS, readCounts);
    sampleTimer.attach_ms(SAMPLE_INTERVAL_MS, sampleWind);
    reportTimer.attach_ms(AVERAGING_PERIOD_MS, reportWind);

    // AS5600 Magnetic angle sensor init
    as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    int b = as5600.isConnected();
    if (b == 1)
    {
        Serial.println("AS5600 Initialized");
    }
    else
    {
        Serial.println("AS5600 Error!");
    }

    //Battery voltage measurement
    pinMode(BATT_VOLTAGE_MEAS_PIN, INPUT);
    pinMode(BATT_VOLTAGE_MEAS_SWITCH, OUTPUT);
    delay(500); // Wait for the sensor to stabilize
}
void readVEML6075(void)
{
    // Read raw and compensation data from the sensor
    rawA = uv.rawUva();
    rawB = uv.rawUvb();
    visibleComp = uv.visibleCompensation();
    irComp = uv.irCompensation();
    // Calculate the simple UVIA and UVIB. These are used to calculate the UVI signal.
    // Kompenzált UV értékek kiszámítása
    float uvac = rawA - (UVA_VIS_COEF_A * visibleComp) - (UVA_IR_COEF_B * irComp);
    float uvbc = rawB - (UVB_VIS_COEF_C * visibleComp) - (UVB_IR_COEF_D * irComp);

    // Ne legyen negatív érték, mert az elrontja az UVI-t
    if (uvac < 0) uvac = 0;
    if (uvbc < 0) uvbc = 0;

    // UV index számítás (mW/cm² → UVI konverzió)
    uvia = uvac * UVA_RESPONSIVITY;
    uvib = uvbc * UVB_RESPONSIVITY;

    // UVI átlagolt
    uvi = (uvia + uvib) / 2.0;
}
void readBH1750(void)
{
    if (lightMeter.measurementReady())
    {
        lux = lightMeter.readLightLevel();
        /*Serial.print("Light: ");
        Serial.print(lux);
        Serial.println(" lx");*/
    }
}
void readBME280(void)
{
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
}
void readQMC5883L(void)
{
    compass.read();
    //azimuth = (compass.getAzimuth() + sensor_orientation_offset);     //TODO if azimuth>360 azimuth=azimuth-360;
    azimuth = compass.getAzimuth();
}
void readWindDirection(void)
{
    windDirection = (as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
}
void printDebugInfo(void)
{
    /*Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("UVIA: "); Serial.print(uvia);
    Serial.print(", UVIB: "); Serial.print(uvib);
    Serial.print(", UVI: "); Serial.println(uvi);

    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lx");
    
    Serial.print("Wind direction: ");
    Serial.print(windDirection);
    Serial.println("°");*/

    Serial.print("Wind speed: ");
    Serial.print(avgSpeed);
    Serial.println("km/h");
    
    /*Serial.print("Rainfall: ");
    Serial.print(rainCount);
    Serial.println("mm");

    Serial.print("Azimuth: ");
    Serial.println(azimuth)
    
    Serial.print("Battery voltage: ");
    Serial.print(batteryVoltage); Serial.println(" mV");*/

    /*Serial.print("MQTT: Send Data -> ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(jsonBuffer);*/
}
ICACHE_RAM_ATTR void lightningISR(void)
{
  lightningDetected = true;
}
void setupWindPCNT(void)
{
    pcnt_config_t pcnt_config =
    {
        .pulse_gpio_num = WIND_SPEED_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,    // felfutó él növel
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = 0,
        .unit = WIND_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(WIND_PCNT_UNIT, 1000);
    pcnt_filter_enable(WIND_PCNT_UNIT);
    pcnt_counter_pause(WIND_PCNT_UNIT);
    pcnt_counter_clear(WIND_PCNT_UNIT);
    pcnt_counter_resume(WIND_PCNT_UNIT);
}
void setupRainPCNT(void)
{
    pcnt_config_t pcnt_config =
    {
        .pulse_gpio_num = RAIN_GAUGE_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,    // felfutó él növel
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = 0,
        .unit = RAIN_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(RAIN_PCNT_UNIT, 1000);
    pcnt_filter_enable(RAIN_PCNT_UNIT);
    pcnt_counter_pause(RAIN_PCNT_UNIT);
    pcnt_counter_clear(RAIN_PCNT_UNIT);
    pcnt_counter_resume(RAIN_PCNT_UNIT);
}
void readCounts(void)
{
    //pcnt_get_counter_value(WIND_PCNT_UNIT, &windCount);
    //pcnt_get_counter_value(RAIN_PCNT_UNIT, &rainCount);

    windDirection = (as5600.rawAngle() * AS5600_RAW_TO_DEGREES);

    // ✏️ Itt számolhatod tovább: windCount → km/h, rainCount → mm / liter stb.
    //pcnt_counter_clear(WIND_PCNT_UNIT);
    pcnt_counter_clear(RAIN_PCNT_UNIT);
}
float calculateWindSpeedKmh(int16_t totalPulses, float intervalSec)
{
    float revolutions = (float)totalPulses / IMPULSES_REV;
    float circumference = 2.0f * PI * RADIUS_M;
    float distance = revolutions * circumference;   // méter / intervallum
    float avgSpeed_m_s = distance / (intervalSec);       // m/s
    float avgSpeed_kmh = avgSpeed_m_s * 3.6f * FRICTION_COMPENSATION; // km/h
    return avgSpeed_kmh;                            // km/h
}
void sampleWind(void)
{
    int16_t pulseCount = 0;
    pcnt_get_counter_value(WIND_PCNT_UNIT, (int16_t *)&pulseCount);
    pcnt_counter_clear(WIND_PCNT_UNIT);

    totalPulses += pulseCount;
    sampleCount++;

    float intervalSec = SAMPLE_INTERVAL_MS / 1000.0f;
    float currentSpeed = calculateWindSpeedKmh(pulseCount, intervalSec);

    if (currentSpeed > maxSpeed)
    {
        maxSpeed = currentSpeed;
    }

    //Serial.printf("Pillanatnyi sebesseg: %.2f km/h\n", currentSpeed);
}
void reportWind(void)
{
    float totalTimeSec = AVERAGING_PERIOD_MS / 1000.0f;  // 600 s
    float revolutions = (float)totalPulses / IMPULSES_REV;
    float circumference = 2.0f * PI * RADIUS_M;
    float distance = revolutions * circumference;   // méter 10 perc alatt
    avgSpeed = (distance / totalTimeSec) * 3.6f; // km/h

    Serial.println("\n===== 10 perces meresi ciklus eredmenye =====");
    Serial.printf("Osszes impulzus: %ld\n", totalPulses);
    Serial.printf("Atlagos szelsebesseg: %.4f km/h\n", avgSpeed);
    Serial.printf("Maximalis szelsebesseg: %.2f km/h\n", maxSpeed);
    Serial.println("==============================================\n");

    // Új ciklushoz nullázás
    totalPulses = 0;
    sampleCount = 0;
    maxSpeed = 0.0f;
}
void batteryVoltageMeasurement(void)
{
    digitalWrite(BATT_VOLTAGE_MEAS_SWITCH, HIGH);
    delay(10);
    batteryVoltage_RawADC = analogRead(BATT_VOLTAGE_MEAS_PIN);
    digitalWrite(BATT_VOLTAGE_MEAS_SWITCH, LOW);
    batteryVoltage = (batteryVoltage_RawADC * 2 * 1.1 * 1000) / 4095; //mV
    /*Serial.print("Battery voltage: ");
    Serial.print(batteryVoltage); Serial.println(" mV");*/

}
void IRAM_ATTR periodicTickerCallback(void)
{
    measurementDue = true;
}
// A tényleges periodikus mérés és MQTT publikálás (ez fut a fő ciklusból, nem megszakításból)
void performPeriodicMeasurement(void)
{
    //Szenzorok olvasása
    readVEML6075();
    readBH1750();
    readBME280();
    readQMC5883L();
    readWindDirection();
    batteryVoltageMeasurement();
    //printDebugInfo();

    // JSON payload építése
    StaticJsonDocument<600> doc;
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["atmospheric_pressure"] = pressure;
    doc["lux"] = lux;
    doc["uvi"] = uvi;
    doc["wind_speed"] = avgSpeed;
    doc["wind_direction"] = windDirection;
    doc["azimuth"] = azimuth;
    doc["rainfall"] = rainCount;


    if (distance > 0)
    {
        doc["lightningdistance"] = distance;
    }
    if (lightEnergy > 0)
    {
        doc["lightningenergy"] = lightEnergy;
    }
    if (noise_flag > 0)
    {
        doc["noise"] = noise_flag;
    }
    if (disturber_flag > 0)
    {
        doc["disturber"] = disturber_flag;
    }

    char jsonBuffer[600];
    serializeJson(doc, jsonBuffer);

    // Publish
    const char *topic = "office/sensor/demo/state";
    if (mqttPubSub.connected())
    {
        mqttPubSub.publish(topic, jsonBuffer);
    }
    else
    {
        Serial.println("MQTT not connected, skipping publish.");
    }

    Serial.print("MQTT: Send Data -> ");
    //Serial.print(topic);
    //Serial.print(": ");
    //Serial.println(jsonBuffer);

    // reset event-specific fields
    distance = 0;
    lightEnergy = 0;
    noise_flag = 0;
    disturber_flag = 0;
}
