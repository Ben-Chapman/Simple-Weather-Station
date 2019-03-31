#include <SPI.h>
#include <Wire.h>
#include "config.h"

#include <ESP8266WiFi.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_EPD.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include "AdafruitIO_Ethernet.h"

// AdafruitIO_Ethernet io(AIO_USERNAME, AIO_KEY);

void setup() {

  Serial.begin(9600);
  Serial.println("Serial setup is working");

//  attempt to connect to Wifi network:
  int status = WL_IDLE_STATUS;     // the Wifi radio's status
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);

    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection
    delay(10000);
 }

 // you're connected now, so print out the data:
 Serial.print("You're connected to the network");
 printWifiData();

   // connect to io.adafruit.com
  // io.connect();

  // wait for a connection
  // while(io.status() < AIO_CONNECTED) {
  //   Serial.print(".");
  //   delay(500);
  // }

  // we are connected
  // Serial.println();
  // Serial.println(io.statusText());


}

void loop() {
  readEnvironmentSensor("temperature");
  Serial.println("Now writing display");
  write_eink_display();
  Serial.println("Entering delay");
  sleep(200);
}

void printWifiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

float readEnvironmentSensor(String sensorType){

  // Initializing the environment sensor
  Adafruit_BME280 bme;  // Using I2C Connections
  bme.begin(&Wire);

  if (sensorType == "temperature" ) {
   float t = bme.readTemperature();  // Degrees C
   Serial.print("Temperature: ");
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
    float p = (bme.readPressure() / 100.0F);  // In hPa
    Serial.print("Pressure: ");
    Serial.println(p);
    return p;
  }
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

  float t = readEnvironmentSensor("temperature");
  float h = readEnvironmentSensor("humidity");
  float p = readEnvironmentSensor("pressure");

  epd.begin();
  epd.setRotation(2); // Landscape
  epd.clearBuffer();
  epd.setTextWrap(false);
  epd.setCursor(5,10);
  epd.setTextSize(2);
  epd.setTextColor(RED_TEXT);
  epd.print("T: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print((1.8 * t) +32);
  epd.println("F");
  epd.setCursor(5,40);
  epd.setTextColor(RED_TEXT);
  epd.print("H: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print(h); epd.println("%");
  epd.setCursor(5,70);
  epd.setTextColor(RED_TEXT);
  epd.print("P: ");
  epd.setTextColor(BLACK_TEXT);
  epd.print((p / 3386.39 ) * 100); epd.println("In");
  epd.setTextColor(BLACK_TEXT);
  epd.setCursor(5,100);
  epd.print("Sig St: ");
  epd.println(WiFi.RSSI());
  epd.setCursor(5,120);
  epd.setTextSize(1.25);
  epd.print("Uptime: "); epd.print(millis());
  epd.display();
}

void adafruit_metrics() {


//   io.run();

//   // save count to the 'counter' feed on Adafruit IO
//   Serial.print("sending -> ");
//   Serial.println(count);
//   counter->save(count);

// // set up the 'temperature' feed
// AdafruitIO_Feed *temperature = io.feed("temperature");

}

void sleep(int sleepTimeInSec) {
  Serial.print("Deep sleeping for ");
  Serial.print(sleepTimeInSec);
  Serial.println(" seconds");
  ESP.deepSleep(sleepTimeInSec * 1000000);  //deepSleep needs microseconds
}
