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
float sensorReadings[5] = {};

WiFiClient client;




// Setup the MQTT client class
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup feeds for publishing.
adafruitIO_feed_setup();

// Adafruit_MQTT_Publish temperaturec_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturec");

// Adafruit_MQTT_Publish temperaturef_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturef");

// Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.humidity");

// Adafruit_MQTT_Publish pressure_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.pressure");

// Adafruit_MQTT_Publish soilmoisture_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.soilmoisture");

// Adafruit_MQTT_Publish runtime_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugruntime");

// Adafruit_MQTT_Publish sigstrength_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugsigstrength");

void MQTT_connect();


void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Serial setup is connected");
  #endif

  // Connect to WiFi
  unsigned long startTime = millis();

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    debugPrint("Attempting to connect to WPA SSID: ");
    debugPrintln(ssid);

    status = WiFi.begin(ssid, pass);

    // wait n seconds for connection
    delay(15000);
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
  float t = readEnvironmentSensor("temperature");  // Degrees C
  float h = readEnvironmentSensor("humidity");  // % Humidity
  float p = readEnvironmentSensor("pressure");  // Barometric pressure
  /*
  The soil moisture sensor returns high values in dry soil (<=786), and progressively lower values for wet soil (>=534). Inversing these values to make them a little more human platable. Wet == bigger number, dry smaller.
  */
  int sm = 1000 - analogRead(A0);  // Soil Moisture

  // Adafruit.io Publishing
  MQTT_connect();

  temperaturec_feed.publish(int(t));  // Degrees C
  temperaturef_feed.publish(int((t * 1.8)+ 32));  // Degrees F
  humidity_feed.publish(int(h));
  pressure_feed.publish(p / 100);  // Millibar
  soilmoisture_feed.publish(sm);
  runtime_feed.publish(int(millis()));
  sigstrength_feed.publish(WiFi.RSSI());

  // Disconnect network to save power while writing display
  mqtt.disconnect();
  WiFi.disconnect();  

  // eInk Display
  debugPrintln("Now writing display");
  write_eink_display(t, h, p, sm);

  debugPrintln("Entering deep sleep...");
  // eInk Display can only refresh 1/180 seconds, so deepSleeping for at least that amount of time
  deepSleep(200);  
}

void adafruitIO_feed_setup() {

  char *adafruitIO_Feeds[7] = {
  "temperaturec",
  "temperaturef",
  "humidity",
  "pressure",
  "soilMoisture",
  "debugruntime",
  "debugsigstrength"
  };

  String feedBasename = "/feeds/weather-station.";
  for(int i = 0; i < sizeof(adafruitIO_Feeds) / sizeof(adafruitIO_Feeds[0]); i++) {
    debugPrintln("Setting up AdafruitIO Feed: ")
    debugPrintln(feedBasename + adafruitIO_Feeds[i]);
   
   Adafruit_MQTT_Publish sigstrength_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME feedBasename + adafruitIO_Feeds[i]);
  }
}

float newreadEnvironmentSensor() {
  // Initializing the environment sensor
  Adafruit_BME280 bme;  // Using I2C Connections
  bme.begin(&Wire);

  sensorReadings[0] = bme.readTemperature();
  sensorReadings[1] = bme.readHumidity();
  sensorReadings[2] = bme.readPressure();

  debugPrint("Temperature C: ");
  debugPrintln(sensorReadings[0]);

  debugPrint("Humidity: ");
  debugPrintln(sensorReadings[1]);

  debugPrint("Pressure in Millibar: ");
  debugPrintln(sensorReadings[2] / 100);
}

float readEnvironmentSensor(String sensorType){
  // Initializing the environment sensor
  Adafruit_BME280 bme;  // Using I2C Connections
  bme.begin(&Wire);

  if (sensorType == "temperature" ) {
   float t = bme.readTemperature();  // Degrees C
   debugPrint("Temperature C: ");
   debugPrintln(t);
   return t;
}
  else if (sensorType == "humidity") {
    float h = bme.readHumidity();
    debugPrint("Humidity: ");
    debugPrintln(h);
    return h;
  }
  else if (sensorType == "pressure") {
    // Pressure is returned in Pascals.
    // 100 Pascal == 1 hPa
    // 1 inHg == 3386.39 Pascal
    float p = (bme.readPressure());  // Pascals
    debugPrint("Pressure in Millibar: ");
    debugPrintln(p / 100);
    return p;
  }
}

void write_eink_display(
  float temperature,
  float humidity,
  float pressure,
  int soilMoisture) {

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
  epd.print(temperature); epd.println(" C");

  epd.setCursor(2,45);
  epd.setTextColor(RED_TEXT);
  epd.print("H: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print(humidity); epd.println(" %");

  epd.setCursor(2,80);
  epd.setTextColor(RED_TEXT);
  epd.print("P: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print(pressure / 3386.39);  // Inches Mercury
  epd.println(" inHg");

  epd.setCursor(2,115);
  epd.setTextColor(RED_TEXT);
  epd.print("SM: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print(soilMoisture);
  epd.print("/466");  // Displaying max value for reference
 
  // Diag info to be displayed at the bottom of the screen
  epd.setTextSize(1.5);
  epd.setTextColor(BLACK_TEXT);
  epd.setCursor(2,137);
  epd.print("Sig: "); epd.print(WiFi.RSSI());
  epd.print(" | IP: "); epd.print(WiFi.localIP());

  epd.display();
}

void deepSleep(int sleepTimeInSec) {
  debugPrint("Deep sleeping for ");
  debugPrint(sleepTimeInSec);
  debugPrintln(" seconds");
  ESP.deepSleep(sleepTimeInSec * 1000000);  //ESP.deepSleep needs microseconds
}

void MQTT_connect();

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  debugPrint("Connecting to MQTT... ");

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