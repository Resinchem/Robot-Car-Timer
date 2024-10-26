/* ===============================================================
   Race Car Timer System
   After successful wifi join, the following can be used from web browser:
    http://ip_address/   - Shows current Device Name, MAC, IPAddress - also list the following commands:
    http://ip_address/restart - Just reboots the board
    http://ip_address/reset - Clears config and relaunches hotspot/config page (can be used to change WiFi or name)
    http://ip_address/otaupdate - puts the system into Arduino OTA mode for wirelessly flashing new firmware
    http://ip_address/leds&brightness=x - change LED brightness.  x = 1 to 10.
    http://ip_address/timer&brightness=x - change timer brightness. x = 1 to 10.

    Version: 0.20  
   ===============================================================*/
//Basic Web and Wifi
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <FS.h>                         //Arduino ESP32 Core - Handles filesystem functions (read/write config file)
#include <LittleFS.h>
#include <ArduinoJson.h>                //Needed for WiFi onboarding: https://arduinojson.org/ 
#include <Wire.h>                       //I2C - distance sensors (ESP Core)
#include <SPI.h>                        //SPI - Timer display (ESP Core)
//OTA Libraries
#include <WiFiUdp.h>
#include <ArduinoOTA.h>                 //OTA Updates: https://github.com/jandrassy/ArduinoOTA
//VL53L0X
#include <VL53L0X.h>                    //VL53L0X ToF Sensor https://github.com/pololu/vl53l0x-arduino  
//MAX7219 Display
#include <MD_MAX72xx.h>                 //MAX7219 Matrix (SPI) https://github.com/JemRF/max7219 (installed as dependency of MD_Parola)
#include <MD_Parola.h>                  //MAX7219 Matrix https://github.com/MajicDesigns/MD_Parola
//LEDs
//LED-related Libraries
#define FASTLED_INTERNAL                // Suppress FastLED SPI/bitbanged compiler warnings
#include <FastLED.h>                    // v3.7.1 LED Strip Control: https://github.com/FastLED/FastLED

#define VERSION "v0.20 (ESP32)"
#define APPNAME "RACECAR TIMER"
#define WIFIMODE 2                      // 0 = Only Soft Access Point, 1 = Only connect to local WiFi network with UN/PW, 2 = Both
#define SERIAL_DEBUG 0                  // 0 = Disable (must be disabled if using RX/TX pins), 1 = enable
#define FORMAT_LITTLEFS_IF_FAILED true

// ======================================================
//  Change or update these values for your build/options
// ======================================================
//Pin Defintions - update if your build is different
#define BUS1_SDA 21                     //I2C Bus 1 Data (ToF start sensor)
#define BUS1_SCL 22                     //I2C Bus 1 Clock (ToF start sensor)
#define BUS2_SDA 17                     //I2C Bus 2 Data (ToF end sensor)
#define BUS2_SCL 19                     //I2C Bus 2 Clock (ToF end sensor)
#define SPI_DIN 23                      //SPI Data In (LED timer matrix )
#define SPI_CLK 18                      //SPI Clock 
#define SPI_CS 5                        //SPI Chip Select
#define LED_DATA_PIN 16                 //Data line for LED strip lighting
#define BUTTON_RESET 26                 //Timer button - Reset
#define BUTTON_START_STOP 25            //Timer button - Manual time start/stop
#define TOGGLE_AUTO 27                  //Toggle switch - auto timing mode
#define TOGGLE_MANUAL 13                //Toggle switch - manual timing mode

//Timing & System Options
#define USE_TENTHS true                   //Setting to false will only show minutes and whole seconds on timer
#define MAX_RACE_MINUTES 5                //Max race time in minutes (10 max when using tenths, 100 otherwise)
#define USE_LEDS true                     //Set to false if not using LED strips (changing this overrides num LEDs)
#define DEFAULT_TIMER_INTENSITY 3         //Matrix timer starting brightness (0-10). Can be changed via URL command.
//Sensor Distances (in mm)
#define START_SENSOR_DIST 406              //Should be distance (mm) from start line sensor to the opposite edge of race track
#define END_SENSOR_DIST 406                //Should be distance (mm) from end line sensor to the opposite edge of race track
//==================================================

//================================================================================
// Do not change any values below unless you are SURE you know what you are doing!
//================================================================================
//Peripheral Defines
//MAX7219 Display
#define MAX_DEVICES 4                      //MAX7219 - Number of controllers
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW  //MAX7219 Matrix
//LED Lighting Strips
#define NUM_LEDS 500                       //Number of LEDs (one side only - max - will be updated via onboarding)
#define MILLIAMP_MAX 15000                 //Max milliamp draw permitted by the LEDs
#define DEFAULT_LED_BRIGHTNESS 125         //LED Strip brightness (0-255) - read from config file and set during onboarding

//Boot indication variables
bool useOnboardLED = true;                 //Use onboard LED to show successful wifi join and config file creation 
int blinkLED = 0;                          //Counter for above.  LED will blink 5 times and remain on if useOnboardLED is true
bool useLEDs = USE_LEDS;                   //LED strips will be disabled if num of LEDs is set to zero during onboarding.

//Variables (wifi/onboarding)
String deviceName = "RaceTimer";        //Default Device Name - 16 chars max, no spaces. Will be updated via onboarding. 
String wifiHostName = deviceName;

bool onboarding = false;                //Set to true if no config file or wifi cannot be joined
String wifiSSID = "";
String wifiPW = "";
byte macAddr[6];                           //Array of device mac address as hex bytes (reversed)
String strMacAddr;                         //Formatted string of device mac address
String baseIPAddress;                      //Device assigned IP Address

//For settings/onboarding (JSON)
int numLEDs = 30;
byte ledBrightness = 125;
byte timerBrightness = DEFAULT_TIMER_INTENSITY;  //This is not part of onboarding, but can be changed via URL command (valid 1-10).
int milliAmpsMax = 8000;
unsigned long maxRaceTime = 599900;              //Just a default starting value.  Actual value updated in Setup, using MAX_TIMER_MINUTES

char wifi_ssid[65];
char wifi_pw[65];
char device_name[17];
char led_count[4];
char led_brightness[4];
char milli_amps_max[6];

//OTA Variables
String otaHostName = deviceName + "_OTA";  //Will be updated by device name from onboarding + _OTA
bool ota_flag = true;                       // Must leave this as true for board to broadcast port to IDE upon boot
uint16_t ota_boot_time_window = 2500;       // minimum time on boot for IP address to show in IDE ports, in millisecs
uint16_t ota_time_window = 20000;           // time to start file upload when ota_flag set to true (after initial boot), in millsecs
uint16_t ota_time_elapsed = 0;              // Counter when OTA active
uint16_t ota_time = ota_boot_time_window;

//Application Variables
bool showTenths = USE_TENTHS;               //true = show tenths of a second, false = show only minutes/seconds
bool systemStandby = false;                 //Toggle - center pos: puts system in standby (stops processes, turns off timer/LEDs)
bool timerAuto = true;                      //Toggle - auto/manual timing     
bool timerRunning = false;
bool timerComplete = false;
bool bootUp = true;
unsigned long prevTime = 0;
unsigned long elapsedTime = 0;

//Manual Start/Stop button debouncing
unsigned long lastDebounceTime = 0;         
unsigned long debounceDelay = 75;           //millisecond delay
int startButtonState;                       //For tracking state
int lastButtonState = HIGH;                 //Set initial state (HIGH = not pressed)

//Timing Mode variables
int oldAutoState;
int oldManualState;
bool poweredDown = false;

//Intialize web server
WebServer server(80);

//Instantiate sensors, display and LEDs
VL53L0X startTimer;
VL53L0X endTimer;
TwoWire bus1 = TwoWire(0);   //I2C Bus 1
TwoWire bus2 = TwoWire(1);     //I2C Bus 2
MD_Parola timerDisplay = MD_Parola(HARDWARE_TYPE, SPI_DIN, SPI_CLK, SPI_CS, MAX_DEVICES);  //SPI LED Display (Type, DIN, SCL, CS, NumDevices)
CRGB LEDs[NUM_LEDS];


void readConfigFile() {

  if (LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {  
    #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
      Serial.println("mounted file system");
    #endif
    if (LittleFS.exists("/config.json")) {
      //file exists, reading and loading
      #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
        Serial.println("reading config file");
      #endif
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
          Serial.println("opened config file");
        #endif
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if ( ! deserializeError ) {

          #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
            Serial.println("\nparsed json");
          #endif
          // Read values here from LittleFS (v0.42 - add defaults for all values in case they don't exist to avoid potential boot loop)
          //DON'T NEED TO STORE OR RECALL WIFI INFO - Written to flash automatically by library when successful connection.
          strcpy(device_name, json["device_name"]|"DefaultDevice");
          strcpy(led_count, json["led_count"]|"30");
          strcpy(led_brightness, json["led_brightness"]|"5");
          strcpy(milli_amps_max, json["milli_amps_max"]|"80000");
          

         //Need to set device and hostnames here
          deviceName = String(device_name);
          wifiHostName = deviceName;
          otaHostName = deviceName + "_OTA";
          numLEDs = (String(led_count)).toInt();
          ledBrightness = (String(led_brightness).toInt()) * 25;  //Web setting is 1 to 10, which will be 25 - 250 (multiply/divide by 25).
          if (ledBrightness > 250) {
            ledBrightness = 250;
          } else if (ledBrightness < 25) {
            ledBrightness = 25;
          }
          milliAmpsMax = (String(milli_amps_max)).toInt();
          onboarding = false;
          if (numLEDs < 1) {
            useLEDs = false;
          } else if (numLEDs > 500) {
            numLEDs = 500;
          }
        } else {
          #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
            Serial.println("failed to load json config");
          #endif
          onboarding = true;
        }
        configFile.close();
      }
    } else {
        // No config file found - set to onboarding
        onboarding = true;
    }
          
    LittleFS.end();    //End - need to prevent issue with OTA updates
  } else {
    //could not mount filesystem
    #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
      Serial.println("failed to mount FS");
      Serial.println("LittleFS Formatted. Restarting ESP.");
      //ESP.restart();
    #endif
    
  }

}

void writeConfigFile(bool restart_ESP) {
  //Write settings to LittleFS (reboot to save)
  char t_device_name[18];
  byte dev_name_len = 18;
  char t_led_count[4];
  char t_led_brightness[4];
  char t_milli_amps_max[6];

  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("Attempting to update boot settings");
  #endif
  if (LittleFS.begin()) {
    //convert values to be saved to char arrays
    deviceName.toCharArray(t_device_name, dev_name_len);  //string
    sprintf(t_led_count, "%u", numLEDs);
    sprintf(t_led_brightness, "%u", (ledBrightness / 25));
    sprintf(t_milli_amps_max, "%u", milliAmpsMax);

    //Create serilized JSON doc
    DynamicJsonDocument doc(1024);
    doc.clear();
    //Add any values to save to JSON document
    doc["device_name"] = t_device_name;
    doc["led_count"] = t_led_count;
    doc["led_brightness"] = t_led_brightness;
    doc["milli_amps_max"] = t_milli_amps_max;

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
      #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
        Serial.println("failed to open config file for writing");
      #endif
      configFile.close();
      return;    
    } else {
      #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
       serializeJson(doc, Serial);
      #endif
      serializeJson(doc, configFile);
      #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
        Serial.println("Settings saved.");
      #endif
      configFile.close();
      LittleFS.end();
      if (restart_ESP) {
        ESP.restart();
      }
    }
  } else {
    //could not mount filesystem
    #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
      Serial.println("failed to mount FS");
    #endif
 
  }
}

/* ------------------------
    WEB PAGES AND HANDLERS
   ------------------------*/
void webMainPage() {
  String mainPage = "<html><head>";
  mainPage += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";  //make page responsive
    
  if (onboarding) {
    //Show portal/onboarding page
    mainPage += "<title>VAR_APP_NAME Onboarding</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }\
    </style>\
    </head>\
    <body>";
    mainPage += "<h1>VAR_APP_NAME Onboarding</h1>";
    mainPage += "Please enter your WiFi information below. These are CASE-SENSITIVE and limited to 64 characters each.<br><br>";
    mainPage += "<form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/onboard\">\
      <table>\
      <tr>\
      <td><label for=\"ssid\">SSID:</label></td>\
      <td><input type=\"text\" name=\"ssid\" maxlength=\"64\" value=\"";
    mainPage += wifiSSID;
    mainPage += "\"></td></tr>\
        <tr>\
        <td><label for=\"wifipw\">Password:</label></td>\
        <td><input type=\"password\" name=\"wifipw\" maxlength=\"64\" value=\"";
    mainPage += wifiPW;
    mainPage += "\"></td></tr><table><br>";
    mainPage += "<b>Device Name: </b>Please give this device a unique name from all other devices on your network, including other installs of VAR_APP_NAME. ";
    mainPage += "This will be used to set the WiFi and OTA hostnames.<br><br>";
    mainPage += "16 alphanumeric (a-z, A-Z, 0-9) characters max, no spaces:";
    mainPage += "<table>\
        <tr>\
        <td><label for=\"devicename\">Device Name:</label></td>\
        <td><input type=\"text\" name=\"devicename\" maxlength=\"16\" value=\"";
    mainPage += deviceName;
    mainPage += "\"></td></tr>";
    mainPage += "<tr>\
         <td><label for=\"ledcount\">Number of LEDs:</label></td>\
         <td><input type=\"number\" min=\"0\" max=\"500\" step=\"1\" name=\"ledcount\" value=\"";
    mainPage += numLEDs;
    mainPage += "\"></td></tr>";
    mainPage += "<tr>\
         <td><label for=\"ledbright\">LED Brightness (1-10):</label></td>\
         <td><input type=\"number\" min=\"1\" max=\"10\" step=\"1\" name=\"ledbright\" value=\"";
    mainPage += (ledBrightness / 25) ;
    mainPage += "\"></td></tr>";
    mainPage += "<tr>\
         <td><label for=\"maxmilliamps\">Max. Milliamps:</label></td>\
         <td><input type=\"number\" min=\"1\" max=\"";
    mainPage += String(MILLIAMP_MAX);
    mainPage += "\" step=\"1\" name=\"maxmilliamps\" value=\"";
    mainPage += milliAmpsMax;
    mainPage += "\"></td></tr>";
    mainPage += "<table><br>";
    mainPage += "<input type=\"submit\" value=\"Submit\">";
    mainPage += "</form>";

  } else {
    //Show normal settings page
    mainPage += "<title>VAR_APP_NAME Main Page</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #0000ff; }\
      table, th, td {\
      border: 1px solid black;\
      border-collapse: collapse;\
    }\
    </style>\
    </head>\
    <body>";
    mainPage += "<H1>VAR_APP_NAME Main Page</H1>";
    mainPage += "Firmware Version: VAR_CURRENT_VER<br><br>";
    mainPage += "<table>";
    mainPage += "<tr><td>Device Name:</td><td>" + deviceName + "</td</tr>";
    mainPage += "<tr><td>WiFi Network:</td><td>" + WiFi.SSID() + "</td</tr>";   
    mainPage += "<tr><td>MAC Address:</td><td>" + strMacAddr + "</td</tr>";
    mainPage += "<tr><td>IP Address:</td><td>" + baseIPAddress + "</td</tr>";
    mainPage += "<tr><td>Number of LEDs:</td><td>" + String(numLEDs) + "</td</tr>";
    mainPage += "<tr><td>LED Brightness:</td><td>" + String(ledBrightness / 25) + "</td</tr>"; 
    mainPage += "<tr><td>Timer Brightness:</td><td>" + String(timerBrightness) + "</td</tr>";
    mainPage += "<tr><td>Max Milliamps:</td><td>" + String(milliAmpsMax) + "</td</tr>"; 
    mainPage +="</table>";
    mainPage += "-------------------------------------------------<br><br>";
    mainPage += "You may issue the following commands via your browser to make changes:<br><br>";
    mainPage += "<table>";
    mainPage += "<tr><td>Restart/Reboot the Controller:</td><td>http://" + baseIPAddress + "/restart</td></tr>";
    mainPage += "<tr><td>Reset Device - Remove all settings (you must onboard again):</td><td>http://" + baseIPAddress + "/reset</td></tr>";
    mainPage += "<tr><td>OTA Update - put device into Arduino OTA update mode:</td><td>http://" + baseIPAddress + "/otaupdate</td></tr>";
    mainPage += "<tr><td>LED Brightness*:</td><td>http://" + baseIPAddress + "/leds?brightness=x  (where x = 1 to 10)</td></tr>";
    mainPage += "<tr><td>Timer Brightness*:</td><td>http://" + baseIPAddress + "/timer?brightness=x  (where x = 1 to 10)</td></tr>";
    mainPage += "</table><br><br>";
    mainPage += "<i>*Changes to brightness values are temporary and will reset when the controller is restarted.</i>";

  }
  mainPage += "</body></html>";
  mainPage.replace("VAR_APP_NAME", APPNAME); 
  mainPage.replace("VAR_CURRENT_VER", VERSION);
  server.send(200, "text/html", mainPage);

}

void handleOnboard() {
   byte count = 0;
   bool wifiConnected = true;
   uint32_t currentMillis = millis();
   uint32_t pageDelay = currentMillis + 5000;  
   String webPage = "";
   //Output web page to show while trying wifi join
    webPage = "<html><head>\
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\ 
    <meta http-equiv=\"refresh\" content=\"1\">";   //make page responsive and refresh once per second
    webPage += "<title>VAR_APP_NAME Onboarding</title>\
      <style>\
        body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }\
      </style>\
      </head>\
      <body>";
    webPage += "<h3>Attempting to connect to Wifi</h3><br>";
    webPage += "Please wait...  If WiFi connection is successful, device will reboot and you will be disconnected from the VAR_APP_NAME AP.<br><br>";  
    webPage += "Reconnect to normal WiFi, obtain the device's new IP address and go to that site in your browser.<br><br>";
    webPage += "If this page does remains after one minute, reset the controller and attempt the onboarding again.<br>";
    webPage += "</body></html>";
    webPage.replace("VAR_APP_NAME", APPNAME); 
    server.send(200, "text/html", webPage);
    while (pageDelay > millis()) {
      yield();
    }
   
   //Handle initial onboarding - called from main page
   //Get vars from web page
   wifiSSID = server.arg("ssid");
   wifiPW = server.arg("wifipw");
   deviceName = server.arg("devicename");
   wifiHostName = deviceName;
   numLEDs = server.arg("ledcount").toInt();

   ledBrightness = ((server.arg("ledbright").toInt()) * 25);
   if (ledBrightness > 250) ledBrightness = 250;

   milliAmpsMax = server.arg("maxmilliamps").toInt();
   if (milliAmpsMax > MILLIAMP_MAX) milliAmpsMax = MILLIAMP_MAX;


   //Attempt wifi connection
  #if defined(ESP8266)  
    WiFi.setSleepMode(WIFI_NONE_SLEEP);  //Disable WiFi Sleep
  #elif defined(ESP32)
    WiFi.setSleep(false);
  #endif
   WiFi.mode(WIFI_STA);
   WiFi.hostname(wifiHostName);
   WiFi.begin(wifiSSID, wifiPW);
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.print("SSID:");
    Serial.println(wifiSSID);
    Serial.print("password: ");
    Serial.println(wifiPW);
    Serial.print("Connecting to WiFi (onboarding)");
  #endif
  while (WiFi.status() != WL_CONNECTED) {
    #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
      Serial.print(".");
    #endif
    // Stop if cannot connect
    if (count >= 60) {
      // Could not connect to local WiFi 
      #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
        Serial.println();
        Serial.println("Could not connect to WiFi during onboarding.");   
      #endif  
      wifiConnected = false;
      break;
    }
    delay(500);
    yield();
    count++;
   
  }
  
  if (wifiConnected) {
    //Save settings to LittleFS and reboot
    writeConfigFile(true);
  }
}

void handleRestart() {
    String restartMsg = "<HTML>\
      </head>\
        <title>Controller Restart</title>\
        <style>\
          body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
        </style>\
      </head>\
      <body>\
      <H1>Controller restarting...</H1><br>\
      <H3>Please wait</H3><br>\
      After the controller completes the boot process, you may click the following link to return to the main page:<br><br>\
      <a href=\"http://";      
    restartMsg += baseIPAddress;
    restartMsg += "\">Return to settings</a><br>";
    restartMsg += "</body></html>";
    server.send(200, "text/html", restartMsg);
    delay(1000);
    ESP.restart();
}

void handleReset() {
    String resetMsg = "<HTML>\
      </head>\
        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
        <title>Controller Reset</title>\
        <style>\
          body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
        </style>\
      </head>\
      <body>\
      <H1>Controller Resetting...</H1><br>\
      <H3>After this process is complete, you must setup your controller again:</H3>\
      <ul>\
      <li>Connect a device to the controller's local access point: VAR_APP_NAME_AP</li>\
      <li>Open a browser and go to: 192.168.4.1</li>\
      <li>Enter your WiFi information and set other default settings values</li>\
      <li>Click Save. The controller will reboot and join your WiFi</li>\
      </ul><br>\
      Once the above process is complete, you can return to the main settings page by rejoining your WiFi and entering the IP address assigned by your router in a browser.<br>\
      You will need to reenter all of your settings for the system as all values will be reset to original defaults<br><br>\
      <b>This page will NOT automatically reload or refresh</b>\
      </body></html>";
    resetMsg.replace("VAR_APP_NAME", APPNAME);
    server.send(200, "text/html", resetMsg);
    delay(1000);
    digitalWrite(2, LOW);
    LittleFS.begin();
    LittleFS.format();
    LittleFS.end();
    delay(1000);
    ESP.restart();
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void handleOTAUpdate() {
  String ota_message = "<h1>VAR_APP_NAME Ready for upload...<h1><h3>Start upload from IDE now</h3>";
  ota_message.replace("VAR_APP_NAME", APPNAME);
  server.send(200, "text/html", ota_message);
  ota_flag = true;
  ota_time = ota_time_window;
  ota_time_elapsed = 0;
}

void handleLEDBrightness() {
  //Handle updating run time setting via http: query string
  //e.g.  http://ip_address/leds?brightness=5
  String parmText;
  int parmValue;
  String errMsg = "";
  String htmlPage = "";
  bool hasError = false;
  if (numLEDs > 0) {
    for (int i = 0; i < server.args(); i++) {
      parmText = server.arg(i);
      if ((server.argName(i) == "brightness") || (server.argName(i) == "Brightness")) {

        if (!isValidNumber(parmText)) {
          hasError = true;
          errMsg = "Non-numeric value entered for brightness. LED brightness unchanged.";
        } else {
          parmValue = parmText.toInt();
          if ((parmValue < 0) || (parmValue > 10)) {
            hasError = true;
            errMsg = "Invalid brightness range.  Valid values are 0 - 10.";
          } else {
            //Set LED brightness
            ledBrightness = (parmValue * 25);
            FastLED.setBrightness(ledBrightness);
            FastLED.show();
          }
        }
      
      } else {
        //no parms 
      }
    }

  } else {
    hasError = true;
    errMsg = "LEDs are disabled in the onboarding setup.  Changes ignored.";
  }
  htmlPage = "<html><head>\
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"; 
  if (!hasError) {
    htmlPage += "<meta http-equiv=\"refresh\" content=\"0; url=http://"; 
    htmlPage += baseIPAddress + "\">";
    htmlPage += "</head><body>";
  } else {
  htmlPage += "<title>VAR_APP_NAME Main Page</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #0000ff; }\
    </style>\
    </head>\
    <body>";
    htmlPage += "<H1>VAR_APP_NAME Current Settings</H1><br>";
    //output error message
    htmlPage += "<br>" + errMsg + "<br>";
  }
  htmlPage += "</body></html>";
  htmlPage.replace("VAR_APP_NAME", APPNAME); 
  server.send(200, "text/html", htmlPage);

}

void handleTimerBrightness() {
  //Handle updating run time setting via http: query string
  //e.g.  http://ip_address/timer?brightness=5
  String parmText;
  int parmValue;
  String errMsg = "";
  String htmlPage = "";
  bool hasError = false;

  for (int i = 0; i < server.args(); i++) {
    parmText = server.arg(i);
    if ((server.argName(i) == "brightness") || (server.argName(i) == "Brightness")) {
      if (!isValidNumber(parmText)) {
        hasError = true;
        errMsg = "Non-numeric value entered for brightness. Timer brightness unchanged.";
      } else {
        parmValue = parmText.toInt();
        if ((parmValue < 0) || (parmValue > 10)) {
          hasError = true;
          errMsg = "Invalid brightness range.  Valid values are 0 - 10.";
        } else {
          //Set LED brightness
          timerBrightness = parmValue;
          timerDisplay.setIntensity(timerBrightness);
        }
      }
    } else {
      //no parms 
    }
  }
  
  htmlPage = "<html><head>\
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"; 
  if (!hasError) {
    htmlPage += "<meta http-equiv=\"refresh\" content=\"0; url=http://"; 
    htmlPage += baseIPAddress + "\">";
    htmlPage += "</head><body>";
  } else {
    htmlPage += "<title>VAR_APP_NAME Main Page</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #0000ff; }\
    </style>\
    </head>\
    <body>";
    htmlPage += "<H1>VAR_APP_NAME Current Settings</H1><br>";
    //output error message
    htmlPage += "<br>" + errMsg + "<br>";
  }
  htmlPage += "</body></html>";
  htmlPage.replace("VAR_APP_NAME", APPNAME); 
  server.send(200, "text/html", htmlPage);

}
// ----------------------------
//  Setup Web Handlers
// -----------------------------
void setupWebHandlers() {
  //Onboarding
  server.on("/", webMainPage);
  server.on("/onboard", handleOnboard);
  server.on("/restart", handleRestart);
  server.on("/reset", handleReset);
  server.on("/leds", handleLEDBrightness);
  server.on("/timer", handleTimerBrightness);
  server.onNotFound(handleNotFound);
  //OTAUpdate
  server.on("/otaupdate", handleOTAUpdate);
}

/* =====================================
    WIFI SETUP 
   =====================================
*/
void setupSoftAP() {
  //for onboarding
  WiFi.mode(WIFI_AP);
  WiFi.softAP(deviceName + "_AP");
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("SoftAP Created");
    Serial.println("Web server starting...");
  #endif
  server.begin();

}

bool setupWifi() {
  byte count = 0;
  //attempt connection
  //if successful, return true else false
  delay(200);
  WiFi.hostname(wifiHostName);
  WiFi.begin();
  while (WiFi.status() != WL_CONNECTED) {
    #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
      Serial.print(".");
    #endif
    // Stop if cannot connect
    if (count >= 60) {
      // Could not connect to local WiFi 
      #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
        Serial.println();
        Serial.println("Could not connect to WiFi.");   
      #endif  
      return false;
    }
    delay(500);
    yield();
    count++;
  } 
  //Successfully connected
  baseIPAddress = WiFi.localIP().toString();
  WiFi.macAddress(macAddr);
  strMacAddr = WiFi.macAddress();
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("Connected to wifi... yay!");
    Serial.print("MAC Address: ");
    Serial.println(strMacAddr);
    Serial.print("IP Address: ");
    Serial.println(baseIPAddress);
    Serial.println("Starting web server...");
  #endif  
  server.begin();
  return true;
}

/* 
   ===============================
    MAIN SETUP
   =============================== 
*/
void setup() {
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.begin(115200);
    Serial.println("Starting setup...");
  #endif
  esp_netif_init();
  setupWebHandlers();
  
  // -----------------------------------------
  //  Captive Portal and Wifi Onboarding Setup
  // -----------------------------------------
  //clean FS, for testing - uncomment next lines ONLY if you wish to wipe current FS
  //LittleFS.format();
  //Remove wifi credentials, for testing - uncomment to wipe stored wifi
  //ESP.eraseConfig();
  // *******************************
  // read configuration from FS json
  // *******************************
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("Starting setup...");
  #endif
  readConfigFile();
  if (onboarding) {
    #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
      Serial.println("Entering Onboarding setup...");
    #endif
    setupSoftAP();
  } else if (!setupWifi()) {
    #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
      Serial.println("Wifi connect failed. Reentering onboarding...");
    #endif
    onboarding = true;
    setupSoftAP();
  } else {
    //Connected to Wifi
    //Rest of normal setup here
    //Only for testing the onboarding
    pinMode(2, OUTPUT);
    
  }

  // Setup OTA Updates
  ArduinoOTA.setHostname(otaHostName.c_str());
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
  });
  ArduinoOTA.begin();
  
  //Push Buttons/Toggle
  pinMode(BUTTON_RESET, INPUT_PULLUP);
  pinMode(BUTTON_START_STOP, INPUT_PULLUP);
  pinMode(TOGGLE_AUTO, INPUT_PULLUP);
  pinMode(TOGGLE_MANUAL, INPUT_PULLUP);
  //Set all initially to HIGH (off)
  digitalWrite(BUTTON_RESET, HIGH);
  digitalWrite(BUTTON_START_STOP, HIGH);
  digitalWrite(TOGGLE_AUTO, HIGH);
  digitalWrite(TOGGLE_MANUAL, HIGH);

  //MAX7219
  timerDisplay.begin();
  timerDisplay.setIntensity(timerBrightness);
  timerDisplay.displayClear();
  timerDisplay.setTextAlignment(PA_LEFT);

  //VL53L0X Sensors
  //Wire.begin();
  bus1.begin(BUS1_SDA, BUS1_SCL, 100000);
  bus2.begin(BUS2_SDA, BUS2_SCL, 100000);
  startTimer.setBus(&bus1);
  startTimer.setTimeout(500);
  startTimer.init();
  startTimer.startContinuous();
  endTimer.setBus(&bus2);
  endTimer.setTimeout(500);
  endTimer.init();
  endTimer.startContinuous();

  //------------------------
  // Setup FastLED and LEDs
  //------------------------
  if (useLEDs) {
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(LEDs, numLEDs);  
    FastLED.setDither(false);
    FastLED.setCorrection(TypicalLEDStrip);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, MILLIAMP_MAX);
    FastLED.setBrightness(ledBrightness);
    fill_solid(LEDs, numLEDs, CRGB::Black);
    FastLED.show();
    //Show setup is complete and test LEDs
    fill_solid(LEDs, numLEDs, CRGB::Red);
    FastLED.show();
    delay(500);
    fill_solid(LEDs, numLEDs, CRGB::Green);
    FastLED.show();
    delay(500);
    fill_solid(LEDs, numLEDs, CRGB::Blue);
    FastLED.show();
    delay(500);
    fill_solid(LEDs, numLEDs, CRGB::Black);
    FastLED.show();
    delay(500);
  }
  //-----------------------------------------
  // Calculate max race time in milliseconds
  //-----------------------------------------
  if ((showTenths) && (MAX_RACE_MINUTES > 10)) {
    maxRaceTime = 599900;
  } else if (MAX_RACE_MINUTES > 100) {
    maxRaceTime = 5999900;
  } else {
    maxRaceTime = (((MAX_RACE_MINUTES - 1) * 60000) + 59900);
  }

  //Set initial timing mode, based on toggle position
  setTimingMode();

  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("Setup complete. Entering main loop...");
  #endif

  
}

void loop() {
  //Handle OTA updates when OTA flag set via HTML call to http://ip_address/otaupdate
  if (ota_flag) {
    showOTA(true);
    uint32_t ota_time_start = millis();
    while (ota_time_elapsed < ota_time) {
      ArduinoOTA.handle();  
      ota_time_elapsed = millis()-ota_time_start;   
      delay(10); 
    }
    ota_flag = false;
    showOTA(false);
  }
 
  //Handle any web calls
  server.handleClient();

  if (!onboarding) {
    //Only execute loop and rest of app if onboarding has been completed
    int startDist = 0;
    int endDist = 0;
    //Blink onboard LED three times then remain on - should only start after successful onboard
    if (useOnboardLED) {
      while (blinkLED < 3) {
        digitalWrite(2, HIGH);
        delay(500);
        yield();
        digitalWrite(2, LOW);
        delay(500);
        yield();
        blinkLED ++;
        #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
          Serial.begin(115200);
          Serial.println("Blink");
        #endif
      }
      digitalWrite(2, HIGH);
    } else {
      digitalWrite(2, LOW);
    } 
    //REST OF MAIN LOOP HERE
    unsigned long currentMillis = millis();
    //Check for state change of toggle switch/timing mode
    int curAuto = digitalRead(TOGGLE_AUTO);
    int curManual = digitalRead(TOGGLE_MANUAL);
    if ((curAuto != oldAutoState) || (curManual != oldManualState)) {
      setTimingMode();
    }
    if (systemStandby) {
      if (!poweredDown) {
        timerDisplay.displayClear();
        timerDisplay.print("Stndby");
        fill_solid(LEDs, numLEDs, CRGB::Black);
        FastLED.show();
        delay(1000);
        timerDisplay.displayClear();
        poweredDown = true;
      }
    } else {
      if ((bootUp) || (poweredDown)) {
        bootUp = false;
        poweredDown = false;
        resetTimer();
      }
      //Check for timer button presses - Currently, only the reset button is checked. Still need code for manual start/stop feature
      if (!digitalRead(BUTTON_RESET)) {
        resetTimer();
        #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
          Serial.println("System Reset");
        #endif
      } else if (!timerAuto) {
        //---- MANUAL TIMING -------- (using pushbutton)
        //Only check start/stop button in manual mode
        int reading = digitalRead(BUTTON_START_STOP);
        //Debounce
        if (reading != lastButtonState) {
          lastDebounceTime = millis(); 
        }
        if ((millis() - lastDebounceTime) > debounceDelay) {
          if (reading != startButtonState) {
            startButtonState = reading;
            if (!startButtonState) {
              if ((!timerRunning) && (!timerComplete)) {
                //Start timer
                startTime();
                prevTime = millis();
                currentMillis = prevTime;
                elapsedTime = 0;
              } else if ((timerRunning) && (!timerComplete)) {
                //Stop timer
                stopTime();
              }
            }
          }
        }
        lastButtonState = reading;
      } else {
        //---- AUTOMATIC TIMING ------- (using sensors)
        //Get sensor readings
        startDist = startTimer.readRangeContinuousMillimeters();
        endDist = endTimer.readRangeContinuousMillimeters();
        //Check for starting sensor break if timer not running
        //System ready/standby: timerRunning FALSE and timerComplete FALSE
        //System in progress: timerRunning TRUE and timerComplete FALSE
        //Race Finished: timerRunning FALSE and timerComplete TRUE
        if ((!timerRunning) && (!timerComplete)) {
          if (startDist < START_SENSOR_DIST) {
            //start timer and turn LEDs to race pattern
            startTime();
            prevTime = millis();
            currentMillis = prevTime;
            elapsedTime = 0;
          }
        } else if ((timerRunning) && (!timerComplete)) {
          if (endDist < END_SENSOR_DIST) {
            stopTime();
          }
        }
      }
      //Update time 
      if ((timerRunning) && (!timerComplete)) {
        if ((showTenths) && ((currentMillis - prevTime) >= 100)) {
          prevTime = currentMillis;
          elapsedTime = elapsedTime + 100;
          updateTimer();
          #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
            Serial.print("Elapsed Time (s): ");
            Serial.print(elapsedTime / 1000);
            Serial.print("  Start Dist: ");
            Serial.print(startDist);
            Serial.print("  End Dist: ");
            Serial.println(endDist);
          #endif
          //}
        } else if ((currentMillis - prevTime) >= 1000) {
          prevTime = currentMillis;
          elapsedTime = elapsedTime + 1000;
          updateTimer();
          #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
            Serial.print("Elapsed Time (ms): ");
            Serial.print(elapsedTime / 1000);
            Serial.print("  Start Dist: ");
            Serial.print(startDist);
            Serial.print("  End Dist: ");
            Serial.println(endDist);
          #endif

        }
      }
    }  
  }
}

// ========================
//  Timer Functions
// ========================
void setTimingMode() {
  //Sets timing mode (standby, auto, manual) based on toggle switch pos
  int curAuto = digitalRead(TOGGLE_AUTO);
  int curManual = digitalRead(TOGGLE_MANUAL);
  if (!curAuto) {
    timerAuto = true;
    systemStandby = false;
  } else if (!curManual) {
    timerAuto = false;
    systemStandby = false;
  } else {
    //Toggle in center pos - both GPIO pins high
    systemStandby = true;
  }
  oldAutoState = curAuto;
  oldManualState = curManual;
}

void startTime() {
  timerDisplay.displayClear();
  timerDisplay.setTextAlignment(PA_LEFT);
  if (showTenths) {
    timerDisplay.print(" 0:00.0");  //v0.12 tenths of a second
  } else {
    timerDisplay.print(" 00:00");
  }
  if (useLEDs) {
    //fill_solid(LEDs, numLEDs, CRGB::Green);
    //FastLED.show();
    showRaceLEDs();
  }
  timerRunning = true;
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("Timer started.");
  #endif
}

void stopTime() {
  //stop timer and turn LEDs red
  if (useLEDs) {
    fill_solid(LEDs, numLEDs, CRGB::Green);
    FastLED.show();
  }
  timerComplete = true;
  timerRunning = false;
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("Timer stopped.");
  #endif

}

void resetTimer() {
  timerRunning = false;
  timerComplete = false;
  if (useLEDs) {
    fill_solid(LEDs, numLEDs, CRGB::Yellow);
    FastLED.show();
  }
  timerDisplay.setTextAlignment(PA_CENTER);
  timerDisplay.displayClear();
  timerDisplay.print("Ready");
}

void updateTimer() {
  String outputTime;
  byte m1 = 0;
  byte m2 = 0;
  byte s1 = 0;
  byte s2 = 0;
  byte t1 = 0;
  if (showTenths) {

    if ((elapsedTime == 0) || (elapsedTime > maxRaceTime)) {  //v0.12 - for tenths display, limit to max 9:59 before wrap around/reset
      //timerDisplay.print("0:00.0");
      //elapsedTime = 0;
      //Show time out 
      timerRunning = false;
      timerComplete = true;
      if (useLEDs) {
        fill_solid(LEDs, numLEDs, CRGB::Red);
        FastLED.show();
      }
      timerDisplay.print("Timeout");
      return;    
    } else {
      unsigned long restMillis = elapsedTime;
      unsigned long minutes = (restMillis / 1000) / 60;
      unsigned long seconds = restMillis / 1000;
      unsigned long tenths = restMillis / 10;
      int remTenths = tenths - (seconds * 100);
      int remSeconds = seconds - (minutes * 60);
      int remMinutes = minutes;
      //int remTenths = restMillis % 10000;
      m1 = remMinutes / 10;
      m2 = remMinutes % 10;  
      s1 = remSeconds / 10;
      s2 = remSeconds % 10;
      t1 = remTenths / 10;
      outputTime = " " + String(m2) + ":" + String(s1) + String(s2) + "." + String(t1);
      timerDisplay.print(outputTime);
    }

  } else {

    if ((elapsedTime == 0) || (elapsedTime > maxRaceTime)) {  //v0.12 - for tenths display, limit to max 9:59 before wrap around/reset
      timerRunning = false;
      timerComplete = true;
      if (useLEDs) {
        fill_solid(LEDs, numLEDs, CRGB::Red);
        FastLED.show();
      }
      timerDisplay.print("Timeout");
      return;    
    } else {
      unsigned long restMillis = elapsedTime;
      unsigned long minutes = (restMillis / 1000) / 60;
      unsigned long seconds = restMillis / 1000;
      int remSeconds = seconds - (minutes * 60);
      int remMinutes = minutes;
      m1 = remMinutes / 10;
      m2 = remMinutes % 10;  
      s1 = remSeconds / 10;
      s2 = remSeconds % 10;
      outputTime = " " + String(m1) + String(m2) + ":" + String(s1) + String(s2);
      timerDisplay.print(outputTime);
    }
  }
}
//================================================
//  LED Functions
//================================================
void clearLEDs() {
  if (useLEDs) {
    fill_solid(LEDs, numLEDs, CRGB::Black);
    FastLED.show();
  }
}

//------------------------------------------------
//  Show OTA Active via Display and LEDs (if used)
//------------------------------------------------
void showOTA(bool show) {
  if (show) {
    if (useLEDs) {
      //fill_solid(LEDs, numLEDs, CRGB::Black);
      clearLEDs();
      //Alternate LED colors using red and green
      //FastLED.setBrightness(ledBrightness);
      for (int i=0; i < (numLEDs-1); i = i + 2) {
        LEDs[i] = CRGB::Red;
        LEDs[i+1] = CRGB::Green;
      }
      FastLED.show();
    }
    timerDisplay.setTextAlignment(PA_CENTER);
    timerDisplay.displayClear();
    timerDisplay.print("Upload");
  } else {
    timerDisplay.displayClear();
    fill_solid(LEDs, numLEDs, CRGB::Black);
    FastLED.show();
    //resetTimer();
  }
}

void showRaceLEDs() {
  if (useLEDs) {  
    int segmentLen = 5;  //increase/decrease this number to change width of blue/white segments
    int i = 0;
    byte remainingLEDs = 0;
    clearLEDs();
    if (numLEDs > 9) { //Only split if there is at least 10 LEDs
      while (i < (numLEDs - (segmentLen + 1))) {
        //Blue LEDs
        fill_solid((LEDs + i), segmentLen, CRGB::Blue);
        i = i + segmentLen;
        //White LEDs
        fill_solid((LEDs + i), segmentLen, CRGB::White);
        i = i + segmentLen;
      }
      //Light any remaining LEDs in blue
      remainingLEDs = (numLEDs - i);
      if (remainingLEDs > 0) {
        fill_solid(LEDs + (numLEDs - remainingLEDs), remainingLEDs, CRGB::Blue);
      }
    } else if (numLEDs > 0) {   //Skip if zero to avoid divide by zero error
      //Just fill with blue
      fill_solid(LEDs, numLEDs, CRGB::Blue);
    }
    FastLED.show();
  }
}
//=============================
//  Misc. Functions
//=============================
boolean isValidNumber(String str){
  for(byte i=0;i<str.length();i++) {
    if(isDigit(str.charAt(i))) return true;
  }
  return false;
}