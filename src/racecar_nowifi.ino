/* ===============================================================
   Race Car Timer System - No Wifi
    This version does not implement any wifi functionality.  For
    this reason, other features such as OTA and URL commands are 
    also unavailable.
    Version: 0.20  
   ===============================================================*/
#include <Wire.h>                       //I2C - distance sensors (ESP Core)
#include <SPI.h>                        //SPI - Timer display (ESP Core)
//VL53L0X
#include <VL53L0X.h>                    //VL53L0X ToF Sensor https://github.com/pololu/vl53l0x-arduino  
//MAX7219 Display
#include <MD_MAX72xx.h>                 //MAX7219 Matrix (SPI) https://github.com/JemRF/max7219 (installed as dependency of MD_Parola)
#include <MD_Parola.h>                  //MAX7219 Matrix https://github.com/MajicDesigns/MD_Parola
//LEDs
//LED-related Libraries
#define FASTLED_INTERNAL                // Suppress FastLED SPI/bitbanged compiler warnings
#include <FastLED.h>                    // v3.7.1 LED Strip Control: https://github.com/FastLED/FastLED

#define VERSION "v0.20 (ESP32 NoWiFi)"
#define APPNAME "RACECAR TIMER"
#define SERIAL_DEBUG 0                  //Set to 1 to enable serial debugging via USB or RX/TX

// ======================================================
//  Change or update these values for your build/options
// ======================================================
//Pin Defintions
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
#define USE_TENTHS true
#define MAX_RACE_MINUTES 5              //Max race time in minutes (9 max when using tenths, 99 otherwise)
//LED Lighting Strips
#define USE_LEDS true;                     //Use LED strips. Set to false if not using LED strips
#define NUM_LEDS 65                        //Number of LEDs (one side only). Set to zero if USE_LEDS is false.
#define MILLIAMP_MAX 10000                 //Max milliamp draw permitted by the LEDs
#define DEFAULT_LED_BRIGHTNESS 125         //LED Strip brightness (0-255)
#define DEFAULT_TIMER_INTENSITY 3          //Matrix timer brightness (0-10)
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

//Boot indication variables
bool useOnboardLED = true;                 //Use onboard LED to show successful boot
int blinkLED = 0;                          //Counter for above.  LED will blink 3 times and remain on if useOnboardLED is true

//Application Variables 
String deviceName = "RaceTimer";           //Default Device Name - 16 chars max, no spaces. Will be updated via onboarding. 
bool showTenths = USE_TENTHS;              //true = show tenths of a second, false = show only minutes/seconds (Set via the #define above)
bool systemStandby = false;                //Toggle - center pos: puts system in standby (stops processes, turns off timer/LEDs)
bool timerAuto = true;                     //Toggle - auto/manual timing     
bool timerRunning = false;                 //True when timer actively running 
bool timerComplete = false;                
bool bootUp = true;
unsigned long prevTime = 0;
unsigned long elapsedTime = 0;
//Local variables, using constants - just for consistency with WiFi version - Change via #defines above
bool useLEDs = USE_LEDS;                   //When false, calls to LED strip will be skipped
int numLEDs = NUM_LEDS;
unsigned long maxRaceTime = 599900;        //Just a holder.  Actual milliseconds calculated in Setup() using MAX_RACE_MINUTE

//Manual Start/Stop button debouncing
unsigned long lastDebounceTime = 0;         
unsigned long debounceDelay = 75;           //millisecond delay
int startButtonState;                       //For tracking state
int lastButtonState = HIGH;                 //Set initial state (HIGH = not pressed)

//Timing Mode variables
int oldAutoState;
int oldManualState;
bool poweredDown = false;

//Instantiate sensors, display and LEDs
VL53L0X startTimer;
VL53L0X endTimer;
TwoWire bus1 = TwoWire(0);   //I2C Bus 1
TwoWire bus2 = TwoWire(1);     //I2C Bus 2
MD_Parola timerDisplay = MD_Parola(HARDWARE_TYPE, SPI_DIN, SPI_CLK, SPI_CS, MAX_DEVICES);  //SPI LED Display (Type, DIN, SCL, CS, NumDevices)
CRGB LEDs[NUM_LEDS];

/* 
   ===============================
    MAIN SETUP
   =============================== 
*/
void setup() {
  #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
    Serial.begin(115200);
    Serial.println("Starting Setup");
  #endif
  //Onboard LED (for boot indicator)
  pinMode(2, OUTPUT);
  //Push Buttons
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
  timerDisplay.setIntensity(DEFAULT_TIMER_INTENSITY);
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
    FastLED.setBrightness(DEFAULT_LED_BRIGHTNESS);
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
    Serial.begin(115200);
    Serial.println("Setup Complete");
  #endif
  
}

void loop() {
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
    //Check for timer button presses - Currentl, only the reset button is checked. Still need code for manual start/stop feature
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
      //Get sensor readings
      startDist = startTimer.readRangeContinuousMillimeters();
      endDist = endTimer.readRangeContinuousMillimeters();
      //Check for starting sensor break if timer not running
      //System ready/standby: timerRunning FALSE and timerComplete FALSE
      //System in progress: timerRunning TRUE and timerComplete FALSE
      //Race Finished: timerRunning FALSE and timerComplete TRUE
      if ((!timerRunning) && (!timerComplete)) {
        if (startDist < START_SENSOR_DIST) {
          //start timer and turn LEDs green
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
          prevTime = millis();
          currentMillis = prevTime;
          elapsedTime = 0;
          timerRunning = true;
          #if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
            Serial.println("Timer started.");
          #endif
        }
      } else if ((timerRunning) && (!timerComplete)) {
        if (endDist < END_SENSOR_DIST) {
          //start timer and turn LEDs green
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
  timerDisplay.displayClear();
  timerDisplay.setTextAlignment(PA_CENTER);
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
      //outputTime = String(m1) + String(m2) + ":" + String(s1) + String(s2);
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

void showRaceLEDs() {
  if (useLEDs) {  
    int segmentLen = 5;  //increase/decrease this number to change width of blue/white segments
    int i = 0;
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
