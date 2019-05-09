// Debug mode on (DEBUG 1) will enable the serial monitor and print informational messages to console.
#define DEBUG 1;

// Adafruit.io Config
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""

// WiFi Config
char ssid[] = "";  // WiFi SSID
char pass[] = "";  // WiFI Password

// Providing for debug-level logging
#ifdef DEBUG
    #define debugPrint(x)    Serial.print(x)
    #define debugPrintln(x)  Serial.println(x)
#else
    #define debugPrint(x)
    #define debugPrintln(x)
#endif