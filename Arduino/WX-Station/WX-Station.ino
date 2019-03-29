#include <SPI.h>
#include <Wire.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include "Adafruit_EPD.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#include <ESP8266WiFi.h>

// Wifi Setup
char ssid[] = "";     //  your network SSID (name)
char pass[] = "";  // your network password
int status  = WL_IDLE_STATUS;     // the Wifi radio's status

//eInk Display Setup
#define EPD_CS     5
#define EPD_DC     9
#define SRAM_CS    4
#define EPD_RESET -1 // Shared with Arduino reset pin
#define EPD_BUSY  -1 // Don't use a pin

// 1.54" Adafruit Tri-Color Display
Adafruit_IL0373 epd(152, 152 ,EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

void setup() {

  Serial.begin(9600);
  Serial.println("\nSerial setup is working");

  // attempt to connect to Wifi network:
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
}

void loop() {

}

void printWifiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void read_temp_sensor() {
  Adafruit_BME280 bme; // I2C

    if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();

}

void write_eink_display() {
  #define BLACK_TEXT EPD_BLACK
  #define RED_TEXT EPD_RED

  epd.begin();
  epd.setRotation(2); // Landscape
  epd.clearBuffer();
  epd.setTextWrap(true);
  epd.setCursor(5, 5);
  epd.setTextSize(1.5);
  epd.setTextColor(BLACK_TEXT);
  epd.println("Current Temperature: ");
  epd.setCursor(5,20);
  epd.print("IP Address: ");
  epd.println(WiFi.localIP());
  epd.setCursor(5,35);
  epd.setTextSize(2);
  epd.setTextColor(RED_TEXT);
  epd.print("Sig St: ");
  epd.println(WiFi.RSSI());
  epd.display();
}

void sleep() {
  ESP.deepSleep(30000000);
}
