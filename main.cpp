#include <Arduino.h>                           // Arduino core library
#include <Wire.h>                              // I2C library
#include <WiFi.h>                              // WiFi library
#include <PubSubClient.h>                      // MQTT library
#include <ArduinoJson.h>                       // JSON library
#include <Adafruit_BME280.h>                   // BME280 sensor 0x76
#include <Credentials.h>                       // Credentials
#include "iAQCoreTwoWire.h"                    // Air quality sensor 0x5A
#include <SparkFun_VEML6075_Arduino_Library.h> // UV Sensor 0x10
#include "SparkFun_AS3935.h"                   // Lightning sensor 0x03
#include <BH1750.h>                            // Light sensor 0x23

//----------------------------------------------------------------------------------------

// Define constants
#define PERIOD_MILLSEC_1000 1000
#define PERIOD_MILLSEC_500 500
#define PERIOD_MILLSEC_250 250

#define AS3935_ADDR 0x03
#define INDOOR 0x12
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01
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
unsigned long Time = 0;
int count = 0;
int mqttCounterConn = 0;
String UniqueId;




// BME280 Sensor
Adafruit_BME280 bme;
float temperature, humidity, pressure;

// Light sensor
BH1750 lightMeter(0x23);
float lux = 0;

// Lightning sensor
SparkFun_AS3935 lightning(AS3935_ADDR); // Interrupt pin for lightning detection
const int lightningInt = 18;
// Values for modifying the IC's settings. All of these values are set to their
// default values.
byte noiseFloor = 1;
byte watchDogVal = 1;
byte spike = 1;
byte lightningThresh = 1;
char message[] = "";
// This variable holds the number representing the lightning or non-lightning
// event issued by the lightning detector.
byte intVal = 0;
byte distance = 0; // Distance to storm in km
byte noise_flag = 0;
byte disturber_flag = 0;
long lightEnergy = 0; // Lightning energy
volatile bool lightningDetected = false;


// UV sensor
VEML6075 uv; // Create a VEML6075 object
// Calibration constants:
// Four gain calibration constants -- alpha, beta, gamma, delta -- can be used to correct the output in
// reference to a GOLDEN sample. The golden sample should be calibrated under a solar simulator.
// Setting these to 1.0 essentialy eliminates the "golden"-sample calibration
const float CALIBRATION_ALPHA_VIS = 1.0; // UVA / UVAgolden
const float CALIBRATION_BETA_VIS = 1.0;  // UVB / UVBgolden
const float CALIBRATION_GAMMA_IR = 1.0;  // UVcomp1 / UVcomp1golden
const float CALIBRATION_DELTA_IR = 1.0;  // UVcomp2 / UVcomp2golden
// Responsivity:
// Responsivity converts a raw 16-bit UVA/UVB reading to a relative irradiance (W/m^2).
// These values will need to be adjusted as either integration time or dynamic settings are modififed.
// These values are recommended by the "Designing the VEML6075 into an application" app note for 100ms IT
const float UVA_RESPONSIVITY = 0.00110; // UVAresponsivity
const float UVB_RESPONSIVITY = 0.00125; // UVBresponsivity
// UV coefficients:
// These coefficients
// These values are recommended by the "Designing the VEML6075 into an application" app note
const float UVA_VIS_COEF_A = 2.22; // a
const float UVA_IR_COEF_B = 1.33;  // b
const float UVB_VIS_COEF_C = 2.95; // c
const float UVB_IR_COEF_D = 1.75;  // d
uint16_t rawA, rawB, visibleComp, irComp;
float uviaCalc, uvibCalc, uvia, uvib, uvi;

//----------------------------------------------------------------------------------------

// Function declarations
void setup_wifi(void);
void MqttReconnect(void);
void MqttReceiverCallback(char *topic, byte *inFrame, unsigned int length);
void InitSensors(void);
void readVEML6075(void);
void readBH1750(void);
void ICACHE_RAM_ATTR lightningISR(void);

//----------------------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.print("Start");

    Wire.begin(21, 22);
    Wire.setClock(100000); // iAQ-Core can operate at a maximum of 100kHz

    

    InitSensors();
    delay(100);

    setup_wifi();
    delay(100);

    mqttPubSub.setServer(mqtt_server, mqttPort);
    mqttPubSub.setCallback(MqttReceiverCallback);
}

void loop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!mqttPubSub.connected())
            MqttReconnect();
        else
            mqttPubSub.loop();
    }
    else
    {
        setup_wifi();
    }

    if (millis() - Time > 10000)
    {
        Time = millis();

        if (count++ == 5)
        {
            count = 0;

            float temperature = bme.readTemperature();
            float humidity = bme.readHumidity();
            float pressure = bme.readPressure() / 100.0F;

            readVEML6075();
            readBH1750();
            
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.println(" °C");

            Serial.print("Humidity: ");
            Serial.print(humidity);
            Serial.println(" %");

            Serial.print("Pressure: ");
            Serial.print(pressure);
            Serial.println(" hPa");

            // JSON payload építése
            StaticJsonDocument<600> doc;
            doc["temperature"] = temperature;
            doc["humidity"] = humidity;
            doc["atmospheric_pressure"] = pressure;
            doc["lux"] = lux;
            doc["uvi"] = uvi;
            
            

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

            distance = 0; // Reset distance for next loop
            lightEnergy = 0; // Reset light energy for next loop
            noise_flag = 0; // Reset noise flag for next loop
            disturber_flag = 0; // Reset disturber flag for next loop
        }
    }
    else if (lightningDetected)
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
            // Lightning! Now how far away is it? Distance estimation takes into
            // account previously seen events.
            distance = lightning.distanceToStorm();
            Serial.print("Approximately: ");
            Serial.print(distance);
            Serial.println("km away!");

            // "Lightning Energy" and I do place into quotes intentionally, is a pure
            // number that does not have any physical meaning.
            lightEnergy = lightning.lightningEnergy();
            Serial.print("Lightning Energy: ");
            Serial.println(lightEnergy);
        }
    }
}











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
void MqttReconnect(void)
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
void MqttReceiverCallback(char *topic, byte *inFrame, unsigned int length)
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
void InitSensors(void)
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
        Serial.println(F("Error initialising BH1750"));
    }


    delay(1000); // Wait for the sensor to stabilize
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

    Serial.print("UVIA: "); Serial.print(uvia);
    Serial.print(", UVIB: "); Serial.print(uvib);
    Serial.print(", UVI: "); Serial.println(uvi);
}
void readBH1750(void)
{
    if (lightMeter.measurementReady())
    {
        lux = lightMeter.readLightLevel();
        Serial.print("Light: ");
        Serial.print(lux);
        Serial.println(" lx");
    }
}
ICACHE_RAM_ATTR void lightningISR(void)
{
  lightningDetected = true;
}
