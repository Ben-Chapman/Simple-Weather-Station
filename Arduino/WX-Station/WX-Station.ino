#include <SPI.h>
#include <Wire.h>
#include "config.h"

#include <ESP8266WiFi.h>

// EPD Support
#include <Adafruit_GFX.h>
#include <Adafruit_EPD.h>

// Env sensor Support
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Adafruit.io Support
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/*
  Global configs
*/

// Creating the array to be used for environmental sensor readings

float measurementData[4] = {};

#define TEMPERATURE measurementData[0]
#define HUMIDITY measurementData[1]
#define PRESSURE measurementData[2]
#define SOIL_MOISTURE measurementData[3]

WiFiClient client;

// Setup the MQTT client class
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup feeds for publishing
Adafruit_MQTT_Publish temperaturec_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturec");

Adafruit_MQTT_Publish temperaturef_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturef");

Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.humidity");

Adafruit_MQTT_Publish pressure_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.pressure");

Adafruit_MQTT_Publish soilmoisture_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.soilmoisture");

Adafruit_MQTT_Publish runtime_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugruntime");

Adafruit_MQTT_Publish sigstrength_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugsigstrength");

void MQTT_connect();

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Serial setup is connected");
  #endif

  // Connect to WiFi
  unsigned long startTime = millis();
  IPAddress ip(10, 0, 0, 201);
  IPAddress gateway(10, 0, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress DNS(10, 0, 0, 100);
  int32_t chan = 11;  // Pre-defined (static) WiFi channel

  WiFi.config(ip, gateway, subnet, DNS);
  delay(100);
  WiFi.begin(ssid, pass, chan);

  // int status = WL_IDLE_STATUS;
  debugPrint("Connecting to ");
  debugPrintln(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    debugPrint(".");
    delay(500);
 }

  unsigned long finishTime = millis();

  debugPrint("You're connected to the network: ");
  debugPrintln(WiFi.localIP());
  debugPrint("WiFi connection took ");
  unsigned long timeTaken = ((finishTime - startTime) / 1000);
  debugPrint(timeTaken);
  debugPrintln(" seconds");
}

void loop() {
  // Read measurements from enviromental sensor
  readEnvironmentSensors();

  // Adafruit.io Publishing
  MQTT_connect();

  temperaturec_feed.publish(int(TEMPERATURE));  // Degrees C
  temperaturef_feed.publish(int((TEMPERATURE * 1.8)+ 32));  // Degrees F
  humidity_feed.publish(int(HUMIDITY));
  pressure_feed.publish(PRESSURE / 100);  // Millibar
  soilmoisture_feed.publish(int(SOIL_MOISTURE));
  runtime_feed.publish(int(millis()));
  sigstrength_feed.publish(WiFi.RSSI());

  /*
  The esp8266 provides for ~nvram via the rtc, which we're using to 
  store the loop count of this sketch, and only writing to the EPD
  display every 20th loop, which is every ~63 minutes
  */
  byte rtcStore[2];
  system_rtc_mem_read(65, rtcStore, 2); //offset is 65

  debugPrint("Existing value in rtc is: ");
  debugPrintln(*rtcStore);

  if (*rtcStore % 20 == 0)
  {
    debugPrintln("...Writing display now...");
    // Writing to the EPD
    write_eink_display();
  }

  (*rtcStore)++;  //increment the value
  debugPrint("New value in rtc is: ");
  debugPrintln(*rtcStore);

  system_rtc_mem_write(65, rtcStore, 2); //offset is 65

  delay(2000);
  debugPrintln("Entering deep sleep...");

  // Sleeping
  deepSleep(300);  
}

float readEnvironmentSensors() {
  // Initializing the environment sensor
  Adafruit_BME280 bme;  // Using I2C Connections
  bme.begin(&Wire);

  TEMPERATURE = bme.readTemperature();
  HUMIDITY = bme.readHumidity();
  PRESSURE = bme.readPressure();
  /*
  The soil moisture sensor returns high values in dry soil (<=786), and progressively lower values for wet soil (>=534). Inversing these values to make them a little more human platable. Wet == bigger number, dry smaller.
  */
  SOIL_MOISTURE = 1000 - analogRead(A0);  // Soil Moisture

  debugPrint("Temperature C: ");
  debugPrintln(TEMPERATURE);

  debugPrint("Humidity: ");
  debugPrintln(HUMIDITY);

  debugPrint("Pressure in Millibar: ");
  debugPrintln(PRESSURE / 100);

  debugPrint("Soil Moisture Level: ");
  debugPrintln(SOIL_MOISTURE);
}

void write_eink_display() {

  //eInk Display Setup
  #define EPD_CS     5
  #define EPD_DC     0
  #define SRAM_CS    4
  #define EPD_RESET -1 // Shared with Arduino reset pin
  #define EPD_BUSY  -1 // Don't use a pin

  // 1.54" Adafruit Tri-Color Display
  Adafruit_IL0373 epd(152, 152 ,EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

  #define BLACK_TEXT EPD_BLACK
  #define RED_TEXT EPD_RED
  
  // Initial display configuration
  epd.begin();
  epd.setRotation(2); // Landscape
  epd.clearBuffer();
  epd.setTextWrap(false);
  epd.setCursor(2,0);
  epd.setTextSize(2);

  epd.setCursor(2,10);
  epd.setTextColor(RED_TEXT);
  epd.print("T: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print(TEMPERATURE); epd.println(" C");

  epd.setCursor(2,45);
  epd.setTextColor(RED_TEXT);
  epd.print("H: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print(HUMIDITY); epd.println(" %");

  epd.setCursor(2,80);
  epd.setTextColor(RED_TEXT);
  epd.print("P: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print(int(PRESSURE / 100)); epd.println(" Mb");  // Millibar

  epd.setCursor(2,115);
  epd.setTextColor(RED_TEXT);
  epd.print("SM: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print(int(SOIL_MOISTURE));
  epd.print("/466");  // Displaying max value for reference
 
  // Diag info to be displayed at the bottom of the screen
  epd.setTextSize(1.5);
  epd.setTextColor(BLACK_TEXT);
  epd.setCursor(2,137);
  epd.print("Sig: "); epd.print(WiFi.RSSI());
  epd.print(" | IP: "); epd.print(WiFi.localIP());

  epd.display();

  digitalWrite(16, LOW);  // Pulling low to cut power to the EPD
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

  uint8_t retries = 3;
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