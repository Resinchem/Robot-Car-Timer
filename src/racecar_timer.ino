/* ===============================================================
   Race Car Timer System
   After successful wifi join, the following can be used from web browser:
    http://ip_address/   - Shows current Device Name, MAC, IPAddress - also list the following commands:
    http://ip_address/restart - Just reboots the board
    http://ip_address/reset - Clears config and relaunches hotspot/config page (can be used to change WiFi or name)
    http://ip_address/otaupdate - puts the system into Arduino OTA mode for wirelessly flashing new firmware
    http://ip_address/leds&brightness=x - change LED brightness.  x = 1 to 10.
    http://ip_address/timer&brightness=x - change timer brightness. x = 1 to 10.

    Version: 0.25
   ===============================================================*/
//Basic Web and Wifi
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <FS.h>                         //Arduino ESP32 Core - Handles filesystem functions (read/write config file)
#include <LittleFS.h>
#include <ArduinoJson.h>                //Needed for WiFi onboarding: https://arduinojson.org/ (v7.2.0)
#include <Wire.h>                       //I2C - distance sensors (ESP Core)
#include <SPI.h>                        //SPI - Timer display (ESP Core)
//OTA and Firmware Update Libraries
#include <WiFiUdp.h>
#include <ArduinoOTA.h>                 //OTA Updates: https://github.com/jandrassy/ArduinoOTA (v1.0.12)
#include <ElegantOTA.h>                 //Web based firmware OTA Update:  https://github.com/ayushsharma82/ElegantOTA/ (v3.1.5)
//VL53L0X
#include <VL53L0X.h>                    //VL53L0X ToF Sensor https://github.com/pololu/vl53l0x-arduino (v1.3.1) 
//MAX7219 Display
#include <MD_MAX72xx.h>                 //MAX7219 Matrix (SPI) https://github.com/JemRF/max7219 (installed as dependency of MD_Parola) (v3.5.1)
#include <MD_Parola.h>                  //MAX7219 Matrix https://github.com/MajicDesigns/MD_Parola (v3.7.3)
//LEDs
//LED-related Libraries
#define FASTLED_INTERNAL                // Suppress FastLED SPI/bitbanged compiler warnings
#include <FastLED.h>                    // v3.7.1 LED Strip Control: https://github.com/FastLED/FastLED (v3.7.1)

#define VERSION "v0.25 (ESP32)"
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

//Timing & System Options - can also be updated/saved via web settings
#define USE_TENTHS true                   //Setting to false will only show minutes and whole seconds on timer
#define MAX_RACE_TIME 599                 //Max race time in seconds (599 max when using tenths, 5999 otherwise)
#define USE_LEDS true                     //Set to false if not using LED strips (changing this overrides num LEDs)
#define DEFAULT_TIMER_INTENSITY 3         //Matrix timer starting brightness (0-10). Can be changed via URL command.
//Sensor Distances (in mm) and debounce
#define START_SENSOR_DIST 500             //Should be distance (mm) from start line sensor to the opposite edge of race track
#define END_SENSOR_DIST 500               //Should be distance (mm) from end line sensor to the opposite edge of race track
#define START_SENSOR_DEBOUNCE 2           //Debounce value for start sensor (valid values 1-5)
#define END_SENSOR_DEBOUNCE 2             //Debounce value for end sensor (valid values 1-5)
//LED Lighting Strips
#define NUM_LEDS 500                       //Number of LEDs (one side only - max - will be updated via onboarding)
#define MILLIAMP_MAX 15000                 //Max milliamp draw permitted by the LEDs
#define DEFAULT_LED_BRIGHTNESS 125         //LED Strip brightness (0-255) - read from config file and set during onboarding
//==================================================

//================================================================================
// Do not change any values below unless you are SURE you know what you are doing!
//================================================================================
//Peripheral Defines
//MAX7219 Display
#define MAX_DEVICES 4                      //MAX7219 - Number of controllers
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW  //MAX7219 Matrix
#define NUMELEMENTS(x) sizeof(x)/sizeof(x[0])  // Number of elements in arrays


//Boot indication variables
bool useOnboardLED = true;                 //Use onboard LED to show successful wifi join and config file creation 
int blinkLED = 0;                          //Counter for above.  LED will blink 5 times and remain on if useOnboardLED is true
bool useLEDs = USE_LEDS;                   //LED strips will be disabled if num of LEDs is set to zero during onboarding.
bool showAddressOnBoot = true;             //Output device IP address on MAX7219 on normal boot

//Local Variables (wifi/onboarding)
String deviceName = "RaceTimer";           //Default Device Name - 16 chars max, no spaces. 
String wifiHostName = deviceName;
String wifiSSID = "";
String wifiPW = "";
byte macAddr[6];                           //Array of device mac address as hex bytes (reversed)
String strMacAddr;                         //Formatted string of device mac address
String baseIPAddress;                      //Device assigned IP Address
bool onboarding = false;                   //Will be set to true if no config file or wifi cannot be joined

//Local Variables (Settings & options)
int numLEDs = 30;
unsigned int ledBrightness = DEFAULT_LED_BRIGHTNESS;
int milliAmpsMax = 8000;
byte timerBrightness = DEFAULT_TIMER_INTENSITY;      //This is not part of onboarding, but can be changed via URL command (valid 1-10).
bool showTenths = USE_TENTHS;                        //true = show tenths of a second, false = show only minutes/seconds
bool invertTimer = false;                            //Flip timer display horizontally (so it can be mounted upside down)
unsigned long maxRaceTime = (MAX_RACE_TIME) * 1000;  //Just a default starting value.  Actual value updated in Setup from config file
unsigned int startSensorDist = START_SENSOR_DIST;    //Max trigger distance for start line sensor (in mm)
unsigned int endSensorDist = END_SENSOR_DIST;        //Max trigger distance for finish line sensor (in mm)

//OTA Variables
String otaHostName = deviceName + "OTA";   //Will be updated by device name from onboarding + _OTA
bool ota_flag = true;                       // Must leave this as true for board to broadcast port to IDE upon boot
uint16_t ota_boot_time_window = 2500;       // minimum time on boot for IP address to show in IDE ports, in millisecs
uint16_t ota_time_window = 20000;           // time to start file upload when ota_flag set to true (after initial boot), in millsecs
uint16_t ota_time_elapsed = 0;              // Counter when OTA active
uint16_t ota_time = ota_boot_time_window;

//Misc. Application Variables
bool systemStandby = false;                 //Toggle - center pos: puts system in standby (stops processes, turns off timer/LEDs)
bool timerAuto = true;                      //Toggle - auto/manual timing     
bool timerRunning = false;
bool timerComplete = false;
bool bootUp = true;
unsigned long prevTime = 0;
unsigned long elapsedTime = 0;

//Sensor Debouncing (longer cables/smaller gauge wire may result in noisy signals)
byte startTriggerCount = 0;
byte endTriggerCount = 0;
byte startTriggerMax = START_SENSOR_DEBOUNCE;  //Start Sensor debounce max count
byte endTriggerMax = END_SENSOR_DEBOUNCE;     //End Sensor debounce max count

//Manual Start/Stop button debouncing
unsigned long lastDebounceTime = 0;         
unsigned long debounceDelay = 75;           //millisecond delay
int startButtonState;                       //For tracking state
int lastButtonState = HIGH;                 //Set initial state (HIGH = not pressed)

//Timing Mode variables
int oldAutoState;
int oldManualState;
bool poweredDown = false;
String lastRaceTime = "0:00.0";

//LED Color Definitions - values can be changed via web settings
CRGB ledColorReady = CRGB::Yellow;
CRGB ledColorRaceSeg1 = CRGB::Blue;
CRGB ledColorRaceSeg2 = CRGB::White;
CRGB ledColorEnd = CRGB::Green;
CRGB ledColorExpired = CRGB::Red;
//LED Color indices for web drop downs (match to colors above from array)
byte webColorReady = 8;     //Yellow
byte webColorRaceSeg1 = 1;  //Blue
byte webColorRaceSeg2 = 0;  //White
byte webColorEnd = 3;       //Green
byte webColorExpired = 7;   //Red

//Define Color Arrays
//If adding new colors, increase array sizes and add to void defineColors()
CRGB ColorCodes[10];
String WebColors[10];

//Intialize web server
WebServer server(80);

//Instantiate sensors, display and LEDs
VL53L0X startTimer;
VL53L0X endTimer;
TwoWire bus1 = TwoWire(0);     //I2C Bus 1
TwoWire bus2 = TwoWire(1);     //I2C Bus 2
MD_Parola timerDisplay = MD_Parola(HARDWARE_TYPE, SPI_DIN, SPI_CLK, SPI_CS, MAX_DEVICES);  //SPI LED Display (Type, DIN, SCL, CS, NumDevices)
CRGB LEDs[NUM_LEDS];

//===========================
// Populate Color Arrays
//===========================
void defineColors() {
  //  Increase array sizes in definitions if adding new
  //  Color must be defined as a CRGB::Named Color or as a CRGB RGB value: CRGB(r, g, b);
   ColorCodes[0] = CRGB::White;
   ColorCodes[1] = CRGB::Blue;
   ColorCodes[2] = CRGB::Cyan;
   ColorCodes[3] = CRGB::Green;
   ColorCodes[4] = CRGB::Magenta;
   ColorCodes[5] = CRGB::Orange;
   ColorCodes[6] = CRGB::Pink;
   ColorCodes[7] = CRGB::Red;
   ColorCodes[8] = CRGB::Yellow;
   ColorCodes[9] = CRGB::Black;
   
   WebColors[0] = "White";
   WebColors[1] = "Blue";
   WebColors[2] = "Cyan";
   WebColors[3] = "Green";
   WebColors[4] = "Magenta";
   WebColors[5] = "Orange";
   WebColors[6] = "Pink";
   WebColors[7] = "Red";
   WebColors[8] = "Yellow";
   WebColors[9] = "Black (off)";
}

//============================
// Read config file from flash (LittleFS)
//============================
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
          deviceName = json["device_name"] | "RaceTimer";
          numLEDs = json["led_count"] | 30;
          ledBrightness = json["led_brightness"] | 125;
          milliAmpsMax = json["milli_amps_max"] | 2500;
          timerBrightness = json["timer_brightness"] | 3;
          showTenths = json["show_tenths"] | true;
          invertTimer = json["invert_timer"] | false;
          maxRaceTime = json["max_race_time"] | 120;
          startSensorDist = json["start_sensor_dist"] | 500; 
          startTriggerMax = json["start_trigger_max"] | 2;
          endSensorDist = json["end_sensor_dist"] | 500;
          endTriggerMax = json["end_trigger_max"] | 2;
          webColorReady = json["color_ready"] | 8;
          webColorRaceSeg1 = json["color_race_seg1"] | 1;
          webColorRaceSeg2 = json["color_race_seg2"] | 0;
          webColorEnd = json["color_end"] | 3;
          webColorExpired = json["color_expired"] | 7; 
          
          ledColorReady = ColorCodes[webColorReady];
          ledColorRaceSeg1 = ColorCodes[webColorRaceSeg1];
          ledColorRaceSeg2 = ColorCodes[webColorRaceSeg2];
          ledColorEnd = ColorCodes[webColorEnd];
          ledColorExpired = ColorCodes[webColorExpired];
        
          wifiHostName = deviceName;
          otaHostName = deviceName + "_OTA";
 
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

//==============================
// Write config file to flash (LittleFS)
//==============================
void writeConfigFile(bool restart_ESP) {
  //Write settings to LittleFS (reboot to save)

  if (LittleFS.begin()) {
    //convert values to be saved to char arrays
    //Create serilized JSON doc
    DynamicJsonDocument doc(1024);
    doc.clear();
    //Add any values to save to JSON document
    doc["device_name"] = deviceName;
    doc["led_count"] = numLEDs;
    doc["led_brightness"] = ledBrightness;
    doc["milli_amps_max"] = milliAmpsMax;
    doc["timer_brightness"] = timerBrightness;
    doc["show_tenths"] = showTenths;
    doc["invert_timer"] = invertTimer;
    doc["max_race_time"] = maxRaceTime;
    doc["start_sensor_dist"] = startSensorDist;
    doc["start_trigger_max"] = startTriggerMax;
    doc["end_sensor_dist"] = endSensorDist;
    doc["end_trigger_max"] = endTriggerMax;
    doc["color_ready"] = webColorReady;
    doc["color_race_seg1"] = webColorRaceSeg1;
    doc["color_race_seg2"] = webColorRaceSeg2;
    doc["color_end"] = webColorEnd;
    doc["color_expired"] = webColorExpired;

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
    </style>\
    </head>\
    <body>";
    mainPage += "<H1>VAR_APP_NAME Settings and Options</H1>";
    mainPage += "Firmware Version: VAR_CURRENT_VER<br><br>";
    mainPage += "<table border=\"1\" >";
    mainPage += "<tr><td>Device Name:</td><td>" + deviceName + "</td</tr>";
    mainPage += "<tr><td>WiFi Network:</td><td>" + WiFi.SSID() + "</td</tr>";   
    mainPage += "<tr><td>MAC Address:</td><td>" + strMacAddr + "</td</tr>";
    mainPage += "<tr><td>IP Address:</td><td>" + baseIPAddress + "</td</tr>";
    mainPage +="</table>";
    mainPage += "-------------------------------------------------<br><br>";
    mainPage += "<button type=\"button\" id=\"btnrace\" style=\"font-size: 16px; border-radius: 12px; width: 130px; height: 30px; background-color: #99f2be;\" ";
    mainPage += "onclick=\"location.href = './managerace';\">Manage Race</button><br><br>";
    mainPage += "Changes made below will be used <b><i>until the controller is restarted</i></b>, unless the box to save the settings as new boot defaults is checked. \
    To test settings, leave the box unchecked and click 'Update'. Once you have settings you'd like to keep, check the box and click 'Update' to write the settings as the new boot defaults. \
    If you want to change wifi settings or the device name, you must use the 'Reset All' command.\
    <b><p style=\"color:red;\">NOTE: Changing any settings below will stop the timer if running and reset the system back to 'Ready' mode!</b></p>\
    <form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/applysettings\">";

    //Sensor Settings
    mainPage += "<b><u>Sensor Settings</u></b><br>\
      Maximum distances (in mm) that the sensors will trigger based on motion.  Should be the distance from sensor to just inside the opposite side of the track or course.<br>\
      Do not change the debounce settings unless you are experiencing false triggers.  See the Wiki/User guide for more info on using the debounce setttings.<br><br>\
      <table border=\"0\">\
      <tr>\
      <td><label for=\"startsensordist\">Start Line Sensor Distance (100-1000):</label></td>\
      <td><input type=\"number\" min=\"100\" max=\"1000\" step=\"1\" name=\"startsensordist\" value=\"";
    mainPage += String(startSensorDist);  
    mainPage += "\">mm</td></tr>\
      <tr>\
      <td><label for=\"starttriggercount\">Start Sensor Debounce Count (1-5):</label></td>\
      <td><input type=\"number\" min=\"1\" max=\"5\" step=\"1\" name=\"starttriggercount\" value=\"";
    mainPage += String(startTriggerMax);
    mainPage += "\"></td></tr>\
      <tr>\
      <td><label for=\"endsensordist\">Finish Line Sensor Distance (100-1000):</label></td>\
      <td><input type=\"number\" min=\"100\" max=\"1000\" step=\"1\" name=\"endsensordist\" value=\"";
    mainPage += String(endSensorDist);
    mainPage += "\">mm</td></tr>\
      <tr>\
      <td><label for=\"endtriggercount\">Finish Sensor Debounce Count (1-5):</label></td>\
      <td><input type=\"number\" min=\"1\" max=\"5\" step=\"1\" name=\"endtriggercount\" value=\"";
    mainPage += String(endTriggerMax);
    mainPage += "\"></td></tr></table><br>";

    //Timer Settings
    mainPage += "<b><u>Timer Settings</u></b><br>\
      Maximum allowable race time is determined by the 'Show Tenths' setting.  When enabled, max race time is 599 seconds.  When not using tenths, max race time is 5,999 seconds.<br><br>\
      <table>\
      <tr>\
      <td><label for=\"timerbrightness\">Timer Brightness (0-10):</label></td>\
      <td><input type=\"number\" min=\"0\" max=\"10\" step=\"1\" name=\"timerbrightness\" value=\"";
    mainPage += String(timerBrightness);
    mainPage += "\"></td></tr>\
      <tr>\
      <td><label for=\"chkinverttimer\">Flip Timer Display:</label></td>\
      <td><input type=\"checkbox\" name=\"chkinverttimer\" value=\"invert\"";

    if (invertTimer) {
      mainPage += " checked";
    }
    mainPage += "></td></tr>\
      <tr>\
      <td><label for=\"chkshowtenths\">Use Tenths Timing:</label></td>\
      <td><input type=\"checkbox\" name=\"chkshowtenths\" value=\"showtenths\"";
    if (showTenths) {
      mainPage += " checked";
    } 
    mainPage += "></td></tr>\
      <tr>\
      <td><label for=\"maxracetime\">Max Allotted Race Time:</label></td>\
      <td><input type=\"number\" min=\"0\" max=\"5999\" step=\"1\" name=\"maxracetime\" value=\"";
    mainPage += String((maxRaceTime)/1000);
    mainPage += "\"> seconds</td</tr>\
      </table><br>";

    //LED Settings
    mainPage += "<b><u>LED Settings</u></b><br>\
      If you are not using LED strips, then set the number of LEDs and brightness levels to 0.  For maximum milliamps, the recommended value is 80% of the peak amps provided by your power supply.<br><br>\
      <table>\
      <tr>\
      <td><label for=\"ledcount\">Number of LED Pixels (0-500):</label></td>\
      <td><input type=\"number\" min=\"0\" max=\"500\" step=\"1\" name=\"ledcount\" value=\"";
    mainPage += String(numLEDs);    
    mainPage += "\"> (0 = no LED use)</td></tr>\
      <tr>\
      <td><label for=\"ledbrightness\">Active LED Brightness (0-10):</label></td>\
      <td><input type=\"number\" min=\"0\" max=\"10\" step=\"1\" name=\"ledbrightness\" value=\"";
    mainPage += String((ledBrightness)/25);
    mainPage += "\"></td></tr>\
      <tr>\
      <td><label for=\"maxmilliamps\">Maximum Milliamps (2000-15000):</label></td>\
      <td><input type=\"number\" min=\"2000\" max=\"15000\" step=\"1\" name=\"maxmilliamps\" value=\"";
    mainPage += String(milliAmpsMax);
    mainPage += "\"></td></tr>";    
    //LED Color drop downs
    mainPage += "<tr>\
      <td><label for=\"readycolor\">Ready Color:</label></td>\
      <td><select name=\"readycolor\">";
      for (byte i = 0; i < NUMELEMENTS(WebColors); i++) {
        mainPage += "<option value=\"" + String(i) + "\"";
        if (i == webColorReady) {
          mainPage += " selected";
        }
        mainPage += ">" + WebColors[i] + "</option>";
      }
    mainPage += "\"></td></tr>\
      <tr>\
      <td><label for=\"racecolor1\">Active Race Color 1:</label></td>\
      <td><select name=\"racecolor1\">";
      for (byte i = 0; i < NUMELEMENTS(WebColors); i++) {
        mainPage += "<option value=\"" + String(i) + "\"";
        if (i == webColorRaceSeg1) {
          mainPage += " selected";
        }
        mainPage += ">" + WebColors[i] + "</option>";
      }
    mainPage += "\"></td></tr>\
      <tr>\
      <td><label for=\"racecolor2\">Active Race Color 2:</label></td>\
      <td><select name=\"racecolor2\">";
      for (byte i = 0; i < NUMELEMENTS(WebColors); i++) {
        mainPage += "<option value=\"" + String(i) + "\"";
        if (i == webColorRaceSeg2) {
          mainPage += " selected";
        }
        mainPage += ">" + WebColors[i] + "</option>";
      }
    mainPage += "\"></td></tr>\
      <tr>\
      <td><label for=\"endcolor\">Completed Race Color:</label></td>\
      <td><select name=\"endcolor\">";
      for (byte i = 0; i < NUMELEMENTS(WebColors); i++) {
        mainPage += "<option value=\"" + String(i) + "\"";
        if (i == webColorEnd) {
          mainPage += " selected";
        }
        mainPage += ">" + WebColors[i] + "</option>";
      }
    mainPage += "\"></td></tr>\
      <tr>\
      <td><label for=\"expiredcolor\">Race Time Expired Color:</label></td>\
      <td><select name=\"expiredcolor\">";
      for (byte i = 0; i < NUMELEMENTS(WebColors); i++) {
        mainPage += "<option value=\"" + String(i) + "\"";
        if (i == webColorExpired) {
          mainPage += " selected";
        }
        mainPage += ">" + WebColors[i] + "</option>";
      }
    mainPage += "\"></td></tr></table><br>";
    
    //Save as boot defaults checkbox
    mainPage += "<b><u>Boot Defaults</u></b><br><br>\
      <input type=\"checkbox\" name=\"chksave\" value=\"save\">Save all settings as new boot defaults (controller will reboot)<br><br>\
      <input type=\"submit\" value=\"Update\">\
      </form>\
      <h2>Controller Commands</h2>\
      <b>Caution</b>: Restart and Reset are executed <i>immediately</i> when the button is clicked.<br>\
      <table border=\"1\" cellpadding=\"10\">\
      <tr>\
      <td><button id=\"btnrestart\" onclick=\"location.href = './restart';\">Restart</button></td><td>This will reboot controller and reload default boot values.</td>\
      </tr><tr>\
      <td><button id=\"btnreset\" style=\"background-color:#FAADB7\" onclick=\"location.href = './reset';\">RESET ALL</button></td><td><b><font color=red>WARNING</font></b>: This will clear all settings, including WiFi! You must complete initial setup again.</td>\
      </tr><tr>\
      <td><button id=\"btnupdate\" onclick=\"location.href = './update';\">Firmware Upgrade</button></td><td><i>BETA:</i> Upload and apply new firmware from a compiled .bin file.</td>\
      </tr><tr>\
      <td><button id=\"btnotamode\" onclick=\"location.href = './otaupdate';\">Arudino OTA</button></td><td>Put system in Arduino OTA mode for approx. 20 seconds to flash modified firmware from IDE.</td>\
      </tr>\
      </table><br>";
    mainPage += "-------------------------------------------------<br><br>";
    mainPage += "You may also issue the following commands directly via your browser to make changes:<br><br>";
    mainPage += "<table>";
    mainPage += "<tr><td>Restart/Reboot the Controller:</td><td>http://" + baseIPAddress + "/restart</td></tr>";
    mainPage += "<tr><td>Reset Device - Remove all settings (you must onboard again):</td><td>http://" + baseIPAddress + "/reset</td></tr>";
    mainPage += "<tr><td>Arduino OTA Update - put device into OTA update mode:</td><td>http://" + baseIPAddress + "/otaupdate</td></tr>";
    mainPage += "<tr><td>LED Brightness*:</td><td>http://" + baseIPAddress + "/leds?brightness=x  (where x = 1 to 10)</td></tr>";
    mainPage += "<tr><td>Timer Brightness*:</td><td>http://" + baseIPAddress + "/timer?brightness=x  (where x = 1 to 10)</td></tr>";
    mainPage += "</table><br>";
    mainPage += "<i>*Changes to brightness values using this method are temporary and will reset when the controller is restarted.</i>";
 
  }
  mainPage += "</body></html>";
  mainPage.replace("VAR_APP_NAME", APPNAME); 
  mainPage.replace("VAR_CURRENT_VER", VERSION);
  server.send(200, "text/html", mainPage);

}

void webManageRace() {
  String racePage = "<html><head>";
  racePage += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";  //make page responsive
  racePage += "<title>VAR_APP_NAME Main Page</title>\
  <style>\
    body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #0000ff; }\
  </style>\
  </head>\
  <body>";
  racePage += "<H1>VAR_APP_NAME Race Management</H1>";
  racePage += "Firmware Version: VAR_CURRENT_VER<br><br>";
  racePage += "<table border=\"1\" >";
  racePage += "<tr><td>Device Name:</td><td>" + deviceName + "</td</tr>";
  racePage += "<tr><td>WiFi Network:</td><td>" + WiFi.SSID() + "</td</tr>";   
  racePage += "<tr><td>MAC Address:</td><td>" + strMacAddr + "</td</tr>";
  racePage += "<tr><td>IP Address:</td><td>" + baseIPAddress + "</td</tr>";
  racePage +="</table>";
  racePage += "-------------------------------------------------<br>";
  racePage += "<button type=\"button\" id=\"btnback\" style=\"font-size: 16px; border-radius: 8px; width: 100px; height: 30px;\" \
   onclick=\"location.href = './';\"><< Back</button><br><br>";
  
  racePage += "<H2>Manage Active Races<H2>\
    <table border=\"0\" style=\"font-size: 24px;\">\
    <tr><td>Last Race Time:</td><td>";
  racePage += lastRaceTime;
  racePage += "&nbsp;&nbsp;</td>\
    <td><button type=\"button\" id=\"btnrefresh\" style=\"font-size: 14px; border-radius: 8px; background-color: #aee8eb; 8px; width: 100px; height: 30px;\" \
    onclick=\"location.href = './managerace';\">Refresh</button></td>\
    <td><button type=\"button\" id=\"btnreset\" style=\"font-size: 14px; border-radius: 8px; background-color: #d7dea9; 8px; width: 100px; height: 30px;\" \
    onclick=\"location.href = './racereset';\">Reset</button></td>\
    </tr></table><br>";

  racePage += "<H3>Manual Timing (beta)</H3>\
    <b>IMPORTANT</b>: Using the button below does <u>NOT</u> place the system into manual timing mode and sensors will remain active. \
    It simply allows you to toggle the running/stopped state of the timer.<br><br>\
    <b><i>Additional Note</i></b>: The web handler may introduce a small lag in toggling the timing state. If you need to run in \
    manual timing mode, it is recommended that you use the toggle and buttons on the timer display.<br><br>";
  
  racePage += "<table border=\"0\"><tr>\
    <td><button type=\"button\" id=\"btntoggle\" style=\"font-size: 14px; border-radius: 8px; background-color: #a9f2a5; 8px; width: 100px; height: 30px;\" \
    onclick=\"location.href = './timertoggle';\">";
    if (timerRunning) {
      racePage += "Stop</button></td><td>&nbsp;Current state: RUNNING</td>";
    } else {
      racePage += "Start</button></td><td>&nbsp;Current state: STOPPED</td>";
    }
  racePage += "</td></tr></table>";  

  
  racePage += "</body></html>";
  racePage.replace("VAR_APP_NAME", APPNAME); 
  racePage.replace("VAR_CURRENT_VER", VERSION);
  server.send(200, "text/html", racePage);
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

void handleSettingsUpdate() {
  //Update local variables
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
  } else {
    //Stop any active races
    timerRunning = false;
    timerComplete = false;
    String saveSettings;
    String flipTimer;
    String useTenths;
    int numLEDsOri = numLEDs;  //Needed to refresh display if LED number changed to lower value
    numLEDs = server.arg("ledcount").toInt();

    if (numLEDs < 0) {
      useLEDs = false;
    } else {
      useLEDs = true;
    }
    ledBrightness = (server.arg("ledbrightness").toInt() * 25);
    if (ledBrightness > 250) ledBrightness = 250;
    milliAmpsMax = server.arg("maxmilliamps").toInt();
    timerBrightness = server.arg("timerbrightness").toInt();
    useTenths = server.arg("chkshowtenths");
    if (useTenths == "showtenths") {
      showTenths = true;
    } else {
      showTenths = false;
    }
    flipTimer = server.arg("chkinverttimer");
    if (flipTimer == "invert") {
      invertTimer = true;
    } else {
      invertTimer = false;
    }
    maxRaceTime = server.arg("maxracetime").toInt();
    if ((maxRaceTime > 599) && (showTenths)) {
      maxRaceTime = 599000;
    } else if (maxRaceTime > 5999) {
      maxRaceTime = 5999000;
    } else {
      maxRaceTime = maxRaceTime * 1000;
    }
    startSensorDist = server.arg("startsensordist").toInt();
    startTriggerMax = server.arg("starttriggercount").toInt();
    endSensorDist = server.arg("endsensordist").toInt();
    endTriggerMax = server.arg("endtriggercount").toInt();

    //LED Colors
    webColorReady = server.arg("readycolor").toInt();
    webColorRaceSeg1 = server.arg("racecolor1").toInt();
    webColorRaceSeg2 = server.arg("racecolor2").toInt();
    webColorEnd = server.arg("endcolor").toInt();
    webColorExpired = server.arg("expiredcolor").toInt(); 

    ledColorReady = ColorCodes[webColorReady];
    ledColorRaceSeg1 = ColorCodes[webColorRaceSeg1];
    ledColorRaceSeg2 = ColorCodes[webColorRaceSeg2];
    ledColorEnd = ColorCodes[webColorEnd];
    ledColorExpired = ColorCodes[webColorExpired];

    saveSettings = server.arg("chksave");
    //Update displays with any new values
    updateDisplays(numLEDsOri);

    //Web page output
    String message = "<html>\
      </head>\
        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
        <title>Current System Settings</title>\
        <style>\
          body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
        </style>\
      </head>\
      <body>\
      <H1>Settings updated!</H1><br>\
      <H3>Current values are:</H3>";
      message += "<u>Sensor Settings</u><br>";
      message += "Start Sensor Max Distance: " + String(startSensorDist) + " mm<br>";
      message += "Start Sensor Debounce Count: " + String(startTriggerMax) + "<br>";
      message += "End Sensor Max Distance: " + String(endSensorDist) + " mm<br>";
      message += "End Sensor Debounce Count: " + String(endTriggerMax) + "<br><br>";

      message += "<u>Timer Settings</u><br>";
      message += "Timer Brightness: " + String(timerBrightness)  + "<br>";
      message += "Flip Timer Display: ";
      if (invertTimer) {
        message += "YES<br>";
      } else {
        message += "No<br>";
      }
      message += "Use Tenths Timing: ";
      if (showTenths) {
        message += "YES<br>";
      } else {
        message += "No<br>";
      }
      message += "Max Race Time Allowed: " + String((maxRaceTime / 1000)) + " seconds<br><br>";

      message += "<u>LED Settings</u><br>";
      message += "Number of LEDs: " + String(numLEDs);
      if (!useLEDs) {
        message += " (LEDs disabled)<br>";
      } else {
        message += "<br>";
        message += "LED Brightness: " + String((ledBrightness/25)) + "<br>";
        message += "Maximum Milliamps: " + String(milliAmpsMax) + "<br>";
        message += "Ready Color: " + WebColors[webColorReady] + "<br>";
        message += "Active Race Color 1: " + WebColors[webColorRaceSeg1] + "<br>";
        message += "Active Race Color 2: " + WebColors[webColorRaceSeg2] + "<br>";
        message += "Completed Race Color: " + WebColors[webColorEnd] + "<br>";
        message += "Race Time Expired Color: " + WebColors[webColorExpired]  + "<br>";
      }
      message += "<br>";
    //If update checked, write new values to flash
      if (saveSettings == "save") {
        message += "<b>New settings saved as new boot defaults.</b> Controller will now reboot.<br>";
        message += "You can return to the settings page after the boot complete (timer will show 'Ready' when boot is finished).<br><br>";
      } else {
        message += "<i>*Current settings are temporary and will reset back to boot defaults when controller is restarted.\
          If you wish to make the current settings the new boot defaults, return to the Settings page and check the box to save the current settings as the new boot values.<br><br>";
      }
      message += "<br><a href=\"http://";
      message += baseIPAddress;
      message += "\">Return to settings</a><br>";
      message += "</body></html>";
      server.send(200, "text/html", message);
      delay(1000);
      yield();
      if (saveSettings == "save") {
        writeConfigFile(true); 
      }
  }
}

void handleRaceReset() {
  resetTimer();
  String page = "<HTML><head>";
  page += "<meta http-equiv=\"refresh\" content=\"0; url='http://" + baseIPAddress + "/managerace'\">";
  page += "'\"><title>Resetting Race Timer</title>\</head><body>";
  page += "Resetting race timer...<br><br>";
  page += "If you are not automatically redirected, follow this to <a href='http://" + baseIPAddress + "/managerace'> return to race management page</a>.";
  page += "</body></html>";
  server.send(200, "text/html", page);
  
}

void handleTimerToggle() {
  if (timerRunning) {
    stopTime();
    endTriggerCount = 0;
  } else {
    timerComplete = false;
    startTime();
    prevTime = millis();
    elapsedTime = 0;
    startTriggerCount = 0;

  }
  String page = "<HTML><head>";
  page += "<meta http-equiv=\"refresh\" content=\"0; url='http://" + baseIPAddress + "/managerace'\">";
  page += "'\"><title>Toggle Timer State</title>\</head><body>";
  page += "Toggling Race Timer...<br><br>";
  page += "If you are not automatically redirected, follow this to <a href='http://" + baseIPAddress + "/managerace'> return to race management page</a>.";
  page += "</body></html>";
  server.send(200, "text/html", page);

}

void handleRestart() {
    String restartMsg = "<HTML>\
      <head>\
        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
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
  String ota_message = "<HTML>\
      <head>\
        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
        <title>Arduino OTA Mode</title>\
        <style>\
          body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
        </style>\
      </head>\
      <body>";
  ota_message += "<h1>VAR_APP_NAME Ready for upload...<h1><h3>Start upload from IDE now</h3><br>";
  ota_message += "If no communication is received after approximately 20 seconds, the system will exit OTA mode and return to normal operation.<br><br>\
      If a firmware update is successfully delivered, the controller will reboot.  You can return to the settings after the boot process completes.<br><br>";
  ota_message += "<a href=\"http://";
  ota_message += baseIPAddress;
  ota_message += "\">Return to settings</a><br>";
  ota_message += "</body></html>";

  ota_message.replace("VAR_APP_NAME", APPNAME);
  server.send(200, "text/html", ota_message);
  ota_flag = true;
  ota_time = ota_time_window;
  ota_time_elapsed = 0;
}

void onOTAEnd (bool success) {
  //Post Web OTA Update
  //This is called by the ElegantOTA setup, found under setupWifi()
  String htmlPage = "<HTML>\
      <head>\
        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
        <title>Firmware Update</title>\
        <style>\
          body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
        </style>\
      </head>\
      <body>";
  if (success) {
    htmlPage += "<H1>Firmware Update Successful!</H1><br><br>";
    htmlPage += "The controller will now reboot.  Once the boot is complete, you can return to the main settings page.<br><br>";

  } else {
    htmlPage += "<H1 style=\"color:red;\">Firmware Update FAILED!!!</H1><br><br>";
    htmlPage += "The original firmware is still installed.<br><br>";
  }
  htmlPage += "<a href=\"http://";
  htmlPage += baseIPAddress;
  htmlPage += "\">Return to settings</a><br>";
  htmlPage += "</body></html>";
  server.send(200, "text/html", htmlPage);
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
            if (ledBrightness > 250) {
              ledBrightness = 250;
            } 
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
  server.on("/managerace", webManageRace);
  server.on("/onboard", handleOnboard);
  server.on("/racereset", handleRaceReset);
  server.on("/timertoggle", handleTimerToggle);
  server.on("/restart", handleRestart);
  server.on("/reset", handleReset);
  server.on("/leds", handleLEDBrightness);
  server.on("/timer", handleTimerBrightness);
  server.on("/applysettings", handleSettingsUpdate);
  server.onNotFound(handleNotFound);
  //OTAUpdate
  server.on("/otaupdate", handleOTAUpdate);
  //Web Firmware (ElegantOTA) update intiated by:
  // http://ip_address/update  (or ./update in code)

}

/* =====================================
    WIFI SETUP 
   =====================================
*/
void setupSoftAP() {
  //for onboarding
  WiFi.mode(WIFI_AP);
  WiFi.softAP(deviceName + "_AP");
  delay(200);  //short delay for AP to start
  //Set config with static IP
  IPAddress Ip(192, 168, 4, 1);
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(Ip, Ip, NMask);
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
  //Web OTA Firmware 
  ElegantOTA.begin(&server);    // Start ElegantOTA
  ElegantOTA.onEnd(onOTAEnd);   //Page to display after update

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
  delay(1000);  
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
  // Setup color arrays
  defineColors();   //This must be done before reading config file
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
  //Set initial timing mode, based on toggle position
  setTimingMode();

  if ((!onboarding) && (showAddressOnBoot)) {
    IPAddress curIP = WiFi.localIP();
    for (byte i=0; i < 4; i++) {
      #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
        Serial.print("IP Address Segment[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.println(curIP[i]);
      #endif
      timerDisplay.print(String(curIP[i]));
      delay(750);
    }
    timerDisplay.displayClear();
  }

  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("Setup complete. Entering main loop...");
  #endif
}

// =====================================
//         *** MAIN LOOP ***
// =====================================
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
    ElegantOTA.loop();    
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
          if (startDist < startSensorDist) {
            startTriggerCount++;
            if (startTriggerCount >= startTriggerMax) {
            //start timer and turn LEDs to race pattern
              startTime();
              prevTime = millis();
              currentMillis = prevTime;
              elapsedTime = 0;
              startTriggerCount = 0;
            }
          } else {
            startTriggerCount = 0;
          }
        } else if ((timerRunning) && (!timerComplete)) {
          if (endDist < endSensorDist) {
            endTriggerCount++;
            if (endTriggerCount >= endTriggerMax) {
              stopTime();
              endTriggerCount = 0;
            }
          } else {
            endTriggerCount = 0;
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
    showRaceLEDs();
  }
  timerRunning = true;
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.println("Timer started.");
  #endif
}

void stopTime() {
  //stop timer and set end LED color
  if (useLEDs) {
    //fill_solid(LEDs, numLEDs, CRGB::Green);
    fill_solid(LEDs, numLEDs, ledColorEnd);
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
    fill_solid(LEDs, numLEDs, ledColorReady);
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
        fill_solid(LEDs, numLEDs, ledColorExpired);
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
        fill_solid(LEDs, numLEDs, ledColorExpired);
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
  //v0.25 - capture time for display on web page
  lastRaceTime = outputTime;
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
    if (blinkLED < 3) {
      //Still in boot up mode
      timerDisplay.displayClear();
      fill_solid(LEDs, numLEDs, CRGB::Black);
      FastLED.show();
    } else {
      resetTimer();
    }
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
        fill_solid((LEDs + i), segmentLen, ledColorRaceSeg1);
        i = i + segmentLen;
        //White LEDs
        fill_solid((LEDs + i), segmentLen, ledColorRaceSeg2);
        i = i + segmentLen;
      }
      //Light any remaining LEDs in blue
      remainingLEDs = (numLEDs - i);
      if (remainingLEDs > 0) {
        fill_solid(LEDs + (numLEDs - remainingLEDs), remainingLEDs, ledColorRaceSeg1);
      }
    } else if (numLEDs > 0) {   //Skip if zero to avoid divide by zero error
      //Just fill with first color
      fill_solid(LEDs, numLEDs, ledColorRaceSeg1);
    }
    FastLED.show();
  }
}
//=============================
//  Misc. Functions
//=============================
void updateDisplays(int origLEDCount) {
  //Called when any settings changes are made and timer/LED strips need to be updated with new values
  timerDisplay.setZoneEffect(0, invertTimer, PA_FLIP_UD);
  timerDisplay.setZoneEffect(0, invertTimer, PA_FLIP_LR);
  timerDisplay.setIntensity(timerBrightness);
  fill_solid(LEDs, origLEDCount, CRGB::Black);
  FastLED.setBrightness(ledBrightness);
  FastLED.show();
  resetTimer();
}

boolean isValidNumber(String str){
  for(byte i=0;i<str.length();i++) {
    if(isDigit(str.charAt(i))) return true;
  }
  return false;
}