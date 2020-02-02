#include <Wire.h>
#include "config.h"

#include <ESP8266WiFi.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_SHT31.h"

// Adafruit.io Support
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Timing
unsigned long sketchStartTime = millis();

Adafruit_SHT31 sht31 = Adafruit_SHT31();

Adafruit_BMP3XX bmp; // I2C

/*
  Global configs
*/

// Creating the array to be used for environmental sensor readings

float measurementData[3] = {};

#define TEMPERATURE measurementData[0]
#define HUMIDITY measurementData[1]
#define PRESSURE measurementData[2]

WiFiClient client;

// Setup the MQTT client class
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup feeds for publishing
Adafruit_MQTT_Publish temperaturec_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturec");
Adafruit_MQTT_Publish temperaturef_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturef");
Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.humidity");
Adafruit_MQTT_Publish pressure_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.pressure");
Adafruit_MQTT_Publish runtime_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugruntime");
Adafruit_MQTT_Publish sigstrength_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugsigstrength");

void MQTT_connect();

void setup() {
    #ifdef DEBUG
        Serial.begin(115200);
        Serial.println("Serial setup is connected");
    #endif

    // Connect to WiFi
    unsigned long wifiStartTime = millis();
    IPAddress ip(10, 0, 0, 202);
    IPAddress gateway(10, 0, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress DNS(10, 0, 0, 100);
    int32_t chan = 11;  // Pre-defined (static) WiFi channel

    WiFi.config(ip, gateway, subnet, DNS);
    delay(10);
    WiFi.begin(ssid, pass, chan);

    debugPrint("Connecting to ");
    debugPrintln(ssid);
    while (WiFi.status() != WL_CONNECTED) {
        debugPrint(".");
        delay(100);
    }

    unsigned long wifiEndTime = millis();

    debugPrint("You're connected to the network: ");
    debugPrintln(WiFi.localIP());
    debugPrint("WiFi connection took ");
    debugPrint(wifiEndTime - wifiStartTime);
    debugPrintln(" ms");
}

void loop() {
    // Read measurements from enviromental sensor
    readEnvironmentSensors();

    // Adafruit.io Publishing
    MQTT_connect();

    // temperaturec_feed.publish(int(TEMPERATURE));  // Degrees C
    // temperaturef_feed.publish(int((TEMPERATURE * 1.8)+ 32));  // Degrees F
    // humidity_feed.publish(int(HUMIDITY));
    // pressure_feed.publish(PRESSURE / 100);  // Millibar
    // soilmoisture_feed.publish(int(SOIL_MOISTURE));
    // runtime_feed.publish(int(millis()));
    // sigstrength_feed.publish(WiFi.RSSI());

    // delay(1000);
    debugPrintln("Entering deep sleep...");
    unsigned long sketchEndTime = millis();
    debugPrint("Total Sketch runtime = ");
    debugPrint(sketchEndTime - sketchStartTime);
    debugPrintln(" ms");
    // Sleeping
    // deepSleep(300); 
    deepSleep(30);
}

float readEnvironmentSensors() {
    // Initializing the sensors
    sht31.begin();
    bmp.begin();

    // Set up oversampling and filter initialization
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

    TEMPERATURE = sht31.readTemperature();
    HUMIDITY = sht31.readHumidity();
    PRESSURE = bmp.readPressure();

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

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  debugPrint("Connecting to MQTT ... ");

  int retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       debugPrintln(mqtt.connectErrorString(ret));
       debugPrintln("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  debugPrintln("MQTT Connected!");
}