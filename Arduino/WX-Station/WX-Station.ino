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

// Global configs
WiFiClient client;

// Setup the MQTT client class
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup a feed for publishing.
Adafruit_MQTT_Publish temperaturec_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturec");

Adafruit_MQTT_Publish temperaturef_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.temperaturef");

Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.humidity");

Adafruit_MQTT_Publish pressure_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.pressure");

Adafruit_MQTT_Publish runtime_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugruntime");

Adafruit_MQTT_Publish sigstrength_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weather-station.debugsigstrength");

void MQTT_connect();

void setup() {

  Serial.begin(9600);
  Serial.println("Serial setup is working");

  // Connect to WiFi
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection
    delay(10000);
 }

 // you're connected now, so print out the data:
 Serial.print("You're connected to the network: ");
 Serial.println(WiFi.localIP());
}

void loop() {
  // Read measurements from enviroment sensor
  float t = readEnvironmentSensor("temperature");  // Degrees C
  float h = readEnvironmentSensor("humidity");  // % Humidity
  float p = readEnvironmentSensor("pressure");  // Barometric pressure

  // Adafruit.io Publishing
  MQTT_connect();
  
  temperaturec_feed.publish(t);  // Degrees C
  temperaturef_feed.publish((t * 1.8)+ 32);  // Degrees F
  humidity_feed.publish(h);
  pressure_feed.publish(p / 100);  // Millibar
  runtime_feed.publish(int(millis()));
  sigstrength_feed.publish(WiFi.RSSI());

// For testing only
int sm = 100;
  // eInk Display
  Serial.println("Now writing display");
  // write_eink_display();
  write_eink_display(t, h, p, sm);

  Serial.println("Entering deep sleep...");
  deepSleep(200);  //eInk Display can only refresh 1/180 seconds, so deepSleeping for at least that amount of time
}

float readEnvironmentSensor(String sensorType){
  // Initializing the environment sensor
  Adafruit_BME280 bme;  // Using I2C Connections
  bme.begin(&Wire);

  if (sensorType == "temperature" ) {
   float t = bme.readTemperature();  // Degrees C
   Serial.print("Temperature C: ");
   Serial.println(t);
   return t;
}
  else if (sensorType == "humidity") {
    float h = bme.readHumidity();
    Serial.print("Humidity: ");
    Serial.println(h);
    return h;
  }
  else if (sensorType == "pressure") {
    // Pressure is returned in Pascals.
    // 100 Pascal == 1 hPa
    // 1 inHg == 3386.39 Pascal
    float p = (bme.readPressure());  // Pascals
    Serial.print("Pressure in Millibar: ");
    Serial.println(p / 100);
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


  int yPosition = 10;
  int leading = 50; // Distance between lines

// Loop through n lines of values you wish to display on the EPD
//   char displayValues[][5] = { "T: ", "H: ", "P: ", "SM: " };
  
//   for( int i = 0; i < sizeof(displayValues); i++ ) {
//     if (i != 0) {
//       int yPosition = yPosition + leading;
//       Serial.print("yPosition: ");
//       Serial.println(yPosition);
//     }

//     epd.setCursor(5,yPosition);
//     epd.setTextColor(RED_TEXT);
//     epd.print(displayValues[i]);
//     epd.setTextColor(BLACK_TEXT);

//     if (strstr(displayValues[i], "T") != NULL) {
//       epd.print(temperature);
//       epd.print(" C");
//       epd.print("/");
//       epd.print((temperature * 1.8)+ 32);
//       epd.println(" F");
//     }
//     else if (strstr(displayValues[i], "H") != NULL) {
//       epd.print(humidity);
//       epd.println(" %");
//     }
//     else if (strstr(displayValues[i], "P") != NULL) {
//       epd.print(pressure);
//       epd.println(" inHg");
//     }
//     else if (strstr(displayValues[i], "SM") != NULL) {
//       epd.print(soilMoisture);
//     }
    
// }  // end loop


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
 
  // Diag info to be displayed at the bottom of the screen
  epd.setTextSize(1.5);
  epd.setTextColor(BLACK_TEXT);
  epd.setCursor(2,137);
  epd.print("Sig: "); epd.print(WiFi.RSSI()); epd.print(" | IP: "); epd.print(WiFi.localIP());

  epd.display();
}

void deepSleep(int sleepTimeInSec) {
  Serial.print("Deep sleeping for ");
  Serial.print(sleepTimeInSec);
  Serial.println(" seconds");
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

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}