#include <Wire.h>
#include "config.h"

#include <ESP8266WiFi.h>

// Sensors support
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_SHT31.h"

// Adafruit.io Support
#include "Adafruit_MQTT_Client.h"

// Instrumentation
unsigned long sketchStartTime = millis();

// Creating the array to be used for environmental sensor readings
float measurementData[3] = {};
#define TEMPERATURE measurementData[0]
#define HUMIDITY measurementData[1]
#define PRESSURE measurementData[2]

// Setup the MQTT client class
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup feeds for publishing
Adafruit_MQTT_Publish temperaturec_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturec");
Adafruit_MQTT_Publish temperaturef_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturef");
Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.humidity");
Adafruit_MQTT_Publish pressure_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.pressure");
Adafruit_MQTT_Publish batteryvoltage_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.batteryvoltage");
Adafruit_MQTT_Publish runtime_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugruntime");
Adafruit_MQTT_Publish sigstrength_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugsigstrength");

void setup() {
    // Start with WiFi disabled to save power
    wifiState(0); 

    #ifdef DEBUG
        Serial.begin(115200);
    #endif

}


void loop() {
    // Read battery voltage level
    // Fully charged battery == 4.190V
    float batteryVoltage = analogRead(A0) * (4.190 / 1024.0);
    debugPrint("Battery Voltage: ");
    debugPrintln(batteryVoltage);
    debugPrintln(analogRead(A0));
    // Read measurements from enviromental sensors
    debugPrintln("Reading sensors...");
    readEnvironmentSensors();

    
    // Enable WiFi
    wifiState(1);

    // Adafruit.io Publishing
    MQTT_connect();

    // Publish Feed Data
    batteryvoltage_feed.publish(batteryVoltage);
    temperaturec_feed.publish(int(TEMPERATURE));  // Degrees C
    temperaturef_feed.publish(int((TEMPERATURE * 1.8)+ 32));  // Degrees F
    humidity_feed.publish(int(HUMIDITY));
    pressure_feed.publish(PRESSURE / 100);  // Millibar
    sigstrength_feed.publish(WiFi.RSSI());

    unsigned long sketchEndTime = millis();
    int sketchRunTime = (sketchEndTime - sketchStartTime);

    debugPrint("Total Sketch runtime = ");
    debugPrint(sketchRunTime);
    debugPrintln(" ms");

    runtime_feed.publish(sketchRunTime);
    delay(500);

    // Shutdown WiFi
    wifiState(0);

    debugPrintln("Entering deep sleep...");
    deepSleep(300);
}


float readEnvironmentSensors() {
    // Initializing the sensors
    Adafruit_SHT31 sht31 = Adafruit_SHT31();
    Adafruit_BMP3XX bmp;
    bmp.begin();
    sht31.begin(0x44);

//     // Set up oversampling and filter initialization
    // bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    // bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);

    bmp.setSensorForcedModeSettings(BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_16X, BMP3_IIR_FILTER_COEFF_1);

    TEMPERATURE = sht31.readTemperature();
    HUMIDITY = sht31.readHumidity();
    
    // Pressure readings
    bmp.performReading();
    PRESSURE = bmp.pressure;
//     // put the sensor in sleep mode
    bmp.setSensorInSleepMode();
  
    // PRESSURE = bmp.pressure;
    debugPrint("Temperature C: ");
    debugPrintln(TEMPERATURE);

    debugPrint("Humidity: ");
    debugPrintln(HUMIDITY);

    debugPrint("Pressure in Millibar: ");
    debugPrintln(PRESSURE / 100);
}


void deepSleep(int sleepTimeInSec) {
    debugPrint("Deep sleeping for ");
    debugPrint(sleepTimeInSec);
    debugPrintln(" seconds");
    ESP.deepSleepInstant(sleepTimeInSec * 1000000);  //ESP.deepSleep needs microseconds
}


void wifiState(int stateType) {
    if (stateType == 0) {
      // Shutdown WiFi chip for power savings
      WiFi.mode(WIFI_OFF);
      WiFi.forceSleepBegin();
  }

  if (stateType == 1) {
      // Wake up the WiFi chip
      WiFi.forceSleepWake();

      // Connect to WiFi
      unsigned long wifiStartTime = millis();
      IPAddress ip(10, 0, 0, 202);
      IPAddress gateway(10, 0, 0, 1);
      IPAddress subnet(255, 255, 255, 0);
      IPAddress DNS(10, 0, 0, 100);
      int chan = 11;  // Pre-defined (static) WiFi channel
      unsigned char bssid[18] = { 0x9A, 0x3B, 0xAD, 0xB4, 0xF6, 0x3A }; // Hardcoding the AP's MAC addr

      WiFi.config(ip, gateway, subnet, DNS);

      WiFi.mode(WIFI_STA);

      // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in flash memory.
      WiFi.persistent(false);
      WiFi.begin(ssid, pass, chan, bssid, true);

      debugPrint("Connecting to ");
      debugPrintln(ssid);
      while (WiFi.status() != WL_CONNECTED) {
          debugPrint(".");
          delay(50);
      }
  }
}


void MQTT_connect() {
  // Handles connection to the MQTT server.
  int mqttConnectStatus;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  debugPrint("Connecting to MQTT ... ");

  int retries = 1;

  while ((mqttConnectStatus = mqtt.connect()) != 0) { // connect will return 0 for connected
       debugPrintln(mqtt.connectErrorString(mqttConnectStatus));
       debugPrintln("Retrying MQTT connection in 1 second...");
       mqtt.disconnect();
       delay(1000);  // wait 5 seconds
       retries--;
  }
  debugPrintln("MQTT Connected!");
}