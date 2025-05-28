/**
 * WALL-E CONTROLLER CODE
 *
 * @file       wall-e.ino
 * @brief      Main Wall-E Controller Sketch
 * @author     Simon Bluett
 * @email      hello@chillibasket.com
 * @copyright  Copyright (C) 2021 - Distributed under MIT license
 * @version    2.9
 * @date       29th May 2021
 *
 * HOW TO USE:
 * 1. Install the Adafruit_PWMServoDriver library
 *    a. In the Arduino IDE, go to Sketch->Include Library->Manage Libraries
 *    b. Search for Adafruit PWM Library, and install the latest version
 * 2. Calibrate the servo motors, using the calibration sketch provided in the
 *    GitHub repository. Paste the calibrated values between lines 369 to 391.
 * 3. Upload the sketch to the micro-controller, and open the serial monitor 
 *    at a baud rate of 115200.
 * 4. Additional instructions and hints can be found at:
 *    https://wired.chillibasket.com/3d-printed-wall-e/
 *
 * Modified for ESP32 Wemos D1 R32 and L298 motor driver, used ESP32 Dev Module,
  PSRam : Disabled, 
  Partition Scheme : No OTA - 2Mb App + 2Mb Spiffs, 
  Flash size : 4Mb
 * added eyebrow control
 * replaced the sound with a DFPlayer mp3 card
 * by hbannw 26th February 2024
 * 16 november 2024 : added LittleFS filesystem support
 * 7 december 2024  : some nice additions by RobAllIsGood, thanks
 *    - Added support for various ESP32 boards (ESP32_C6, ESP32_C3,ESP32_D1_Mini, ESP32_WROVER)
 *    - added #ifdef around LED_PIN, if not define dont use it (for boards with very few GPIO
 *    - corrected mySoftwareSerial.begin(9600, SERIAL_8N1,DFPLAYER_RX_PIN , DFPLAYER_TX_PIN); // speed, type, RX, TX, the pins where hardcoded instead of using the #define
 *    - increased default volume to 25
  */

// Comment  the next line tu use SPIFFS
#define USE_LITTLEFS

#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h> // install AsyncTCP from ESP32Async for IDE 2.3.6 and above
#include <ESPAsyncWebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include "Queue.hpp"
#include "MotorControllerL298.hpp"
#include "DFRobotDFPlayerMini.h"
#include "settings.h"

#ifdef USE_LITTLEFS
#include "LittleFS.h"
#else
#include "SPIFFS.h"
#endif


// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// ONLY USE 1 DEFINITION
#define ESP32_WROOM
// #define ESP32_C6_WROOM_DEV
// #define ESP32_C3_SEEED
// #define ESP32_D1_MINI
// #define ESP32_WROVER

// pin mapping for ESP32 WROOM

#ifdef ESP32_WROOM
// 12v Motors with L298N
#define DIRA1 17
#define DIRB1 16
#define ENABLE1 12
#define DIRA2 27
#define DIRB2 14
#define ENABLE2 13

// PCA9685
#define SERVO_ENABLE_PIN 18  // Servo shield output enable pin
#define PCA_SDA          21
#define PCL_SCL          22
// #define PCL_VCC33V
// #define PCL_GND

// DFPlayer Communication
#define DFPLAYER_RX_PIN 25  // use Resitors 1k-gnd, 660, VCC
#define DFPLAYER_TX_PIN 26

//#define AUDIO_OUTPUT_PIN 25

// Comment out if no pin available
#define LED_PIN 12  // use led on the PWM board

// battery pin, only used if enabled below
#define BATTERY_LEVEL_PIN A2

#endif

// -- -- -- -- -- -- -- -- -- -- -- -- -- --

// pin mapping for ESP32 C6 WROOM DEV
#ifdef ESP32_C6_WROOM_DEV
// 12v Motors with L298N
#define DIRA1 18  
#define DIRB1 19  
#define ENABLE1 20 //Changed
#define DIRA2 21 //Changed
#define DIRB2 22  //Changed
#define ENABLE2 23  //Changed

// PCA9685
#define SERVO_ENABLE_PIN 1  // Servo shield output enable pin, not required ?
#define PIN_SDA  6   // used without definition
#define PIN_SCL  7   // used without definition
// #define PCL_VCC33V
// #define PCL_GND

// DFPlayer Communication
#define DFPLAYER_RX_PIN 10  // Changed   // use Resitors 1k-gnd, 660, VCC
#define DFPLAYER_TX_PIN 11
 
// #define AUDIO_OUTPUT_PIN 25  // TBC, not used?

// Comment out if no pin available
#define LED_PIN 9  // Not onboard


// battery pin, only used if enabled below
// Assume GPIO04 A1 CH5, TBC
#define BATTERY_LEVEL_PIN A1  // tbc

#endif

// -- -- -- -- -- -- -- -- -- -- -- -- -- --

// pin mapping for ESP32_C3_SEEED
#ifdef ESP32_C3_SEEED
// 12v Motors with L298N
#define DIRA1 D0  
#define DIRB1 D1  
#define ENABLE1 D2 //Changed
#define DIRA2 D3 //Changed
#define DIRB2 D8  //Changed
#define ENABLE2 D10  //Changed

// PCA9685
#define SERVO_ENABLE_PIN D9  // Servo shield output enable pin, not required ?
#define PIN_SDA  D4   // used without definition
#define PIN_SCL  D5   // used without definition
// #define PCL_VCC33V
// #define PCL_GND

// DFPlayer Communication
#define DFPLAYER_RX_PIN D7  // Changed   
#define DFPLAYER_TX_PIN D6  // use Resitors 1k-gnd, 660, VCC

// #define AUDIO_OUTPUT_PIN 25  // TBC, not used?

// Comment out if no pin available
// #define LED_PIN 1

// battery pin, only used if enabled below
// Assume GPIO04 A1 CH5, TBC
// #define BATTERY_LEVEL_PIN A2  // tbc

#endif

// -- -- -- -- -- -- -- -- -- -- -- -- -- --

// pin mapping for ESP32_D1_MINI
#ifdef ESP32_D1_MINI
// 12v Motors with L298N
#define DIRA1 32  
#define DIRB1 12  
#define ENABLE1 4 //Changed
#define DIRA2 15 //Changed
#define DIRB2 27  //Changed
#define ENABLE2 0  //Changed

// PCA9685
#define SERVO_ENABLE_PIN 26  // Servo shield output enable pin, not required ?
#define PIN_SDA  21   // used without definition
#define PIN_SCL  22   // used without definition


// DFPlayer Communication
#define DFPLAYER_RX_PIN 16  // Changed   
#define DFPLAYER_TX_PIN 17  // use Resitors 1k-gnd, 660, VCC

// #define AUDIO_OUTPUT_PIN 25  // TBC, not used?

// Comment out if no pin available
#define LED_PIN 2

// battery pin, only used if enabled below
// Assume GPIO04 A1 CH5, TBC
#define BATTERY_LEVEL_PIN 36  // tbc

#endif

// -- -- -- -- -- -- -- -- -- -- -- -- -- --

// pin mapping for ESP32_WROVER
#ifdef ESP32_WROVER
// 12v Motors with L298N
#define DIRA1 12  
#define DIRB1 13  
#define ENABLE1 14 //Changed
#define DIRA2 25 //Changed
#define DIRB2 26  //Changed
#define ENABLE2 27  //Changed

// PCA9685
#define SERVO_ENABLE_PIN 23  // Servo shield output enable pin, not required ?
#define PIN_SDA  21   // used without definition
#define PIN_SCL  22   // used without definitionDFPLAYER_RX_PIN
// #define PCL_VCC33V
// #define PCL_GND

// DFPlayer Communication
#define DFPLAYER_RX_PIN 18  // Changed   // use Resitors 1k-gnd, 660, VCC
#define DFPLAYER_TX_PIN 19

// #define AUDIO_OUTPUT_PIN 25  //  Not used


// Comment out if no pin available
// #define LED_PIN 2  // Not onboard


// battery pin, only used if enabled below
// Assume GPIO04 A1 CH5, TBC
// #define BATTERY_LEVEL_PIN A2  // tbc

#endif

// -- -- -- -- -- -- -- -- -- -- -- -- -- --


// Motors with L298N settings
#define PROG_RUN
#define MAX_SPEED 240       // sets speed of DC  motors
#define MAX_BACK_SPEED 240  // sets speed of DC  motors for backward movement
#define MAX_SPEED_OFFSET 0  // @ 20 Difference between left and right motor
#define turn_amount 750


// DFPlayer Speed
#define NORMAL_SPEED 1  // These are the playback speeds, change to
#define FAST_SPEED 1.5  // see the effect on the sound sample. 1 is default speed
#define SLOW_SPEED 2    // 0.75  // >1 faster, <1 slower, 2 would be twice as fast, 0.5 half as fast

/**
 * Battery level detection
 *
 *   .------R1-----.-------R2------.     | The diagram to the left shows the  |
 *   |             |               |     | potential divider circuit used by  |
 * V_Raw     Analogue pin Ax      GND    | the battery level detection system |
 *
 * @note The scaling factor is calculated according to ratio of the two resistors:
 *       DIVIDER_SCALING_FACTOR = R2 / (R1 + R2)
 *       For example: 47000 / (100000 + 47000) = 0.3197
 *
 * To enable battery level detection, uncomment the next line:
 */
//#define BAT_L
#ifdef BAT_L
#define BATTERY_MAX_VOLTAGE 12.6
#define BATTERY_MIN_VOLTAGE 10.2
#define DIVIDER_SCALING_FACTOR 0.3197


/**
	 * OLED Battery Level Display
	 *
	 * Displays the battery level on an oLed display. Supports a 1.3 inch oLed display using I2C.
	 * The constructor is set to a SH1106 1.3 inch display. Change the constructor if you want to use a different display.
	 * 
	 * @note Requires Battery level detection to be enabled above
	 * @note You may get a "Low memory available" warning when compiling for Arduino UNO boards (79% memory usage).
	 *       It did work in my case, so you should be able to ignore this message.
	 *
	 * To enable the oLED display, uncomment the next line:
	 */
//#define OLED
#ifdef OLED

#include <U8g2lib.h>
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, 10);

#endif /* OLED */
#endif /* BAT_L */


/// Define other constants
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define NUMBER_OF_SERVOS 9       // Number of servo motors, 7 without eyelids, 9 with
#define SERVO_UPDATE_TIME 10     // Time in milliseconds of how often to update servo and motor positions
#define SERVO_OFF_TIME 6000      // Turn servo motors off after 6 seconds
#define STATUS_CHECK_TIME 10000  // Time in milliseconds of how often to check robot status (eg. battery level)
#define CONTROLLER_THRESHOLD 1   // The minimum error which the dynamics controller tries to achieve
#define MAX_SERIAL_LENGTH 5      // Maximum number of characters that can be received

/// Instantiate Objects
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Servo shield controller class - assumes default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Set up motor controller classes
// For Arduino motor shield
//MotorController motorL(DIRECTION_L_PIN, PWM_SPEED_L_PIN, BRAKE_L_PIN, false);
//MotorController motorR(DIRECTION_R_PIN, PWM_SPEED_R_PIN, BRAKE_R_PIN, false);
// Version for L298 controller
MotorController motorL(DIRA1, ENABLE1, DIRB1, false);
MotorController motorR(DIRA2, ENABLE2, DIRB2, false);

// SpeedArray contains the order in which the code will playback the sample at the designated speeds
float SpeedArray[] = { NORMAL_SPEED, FAST_SPEED, SLOW_SPEED };
uint8_t NoOfSpeeds = 1;  // Number of difference speeds in the Speed array above
uint8_t SpeedIdx = 0;    // In effect when the checks in the main loop are made this will increment to 0
boolean soundPlaying = false;


boolean isSound = true;  // Assume sound effects are available
// Create Serial comm and DFPlayer

HardwareSerial mySoftwareSerial(1);

DFRobotDFPlayerMini myDFPlayer;


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Queue for animations - buffer is defined outside of the queue class
// so that the compiler knows how much dynamic memory will be used
struct animation_t {
  uint16_t timer;
  int8_t servos[NUMBER_OF_SERVOS];
};

#define QUEUE_LENGTH 50
animation_t buffer[QUEUE_LENGTH];
Queue<animation_t> queue(QUEUE_LENGTH, buffer);


/// Motor Control Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int pwmspeed = 240;
int moveValue = 0;
int turnValue = 0;
int turnOffset = 0;
int motorDeadzone = 0;
int oldCurvel = 0;

/// Runtime Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
unsigned long lastTime = 0;
unsigned long animeTimer = 0;
unsigned long motorTimer = 0;
unsigned long statusTimer = 0;
unsigned long updateTimer = 0;
bool autoMode = false;


// Serial Parsing
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
char firstChar;
char serialBuffer[MAX_SERIAL_LENGTH];
uint8_t serialLength = 0;


// ****** SERVO MOTOR CALIBRATION *********************
// Servo Positions:  Low,High
int preset[][2] = { { 458, 262 },    // head rotation
                    { 565, 98 },     // neck top
                    { 170, 570 },    // neck bottom
                    { 215, 436 },    // eye right
                    { 294, 135 },    // eye left
                    { 350, 185 },    // arm left
                    { 188, 360 },    // arm right
                    { 210, 150 },    // eyebrow right
                    { 307, 375 } };  // eyebrow left
// *****************************************************


// Servo Control - Position, Velocity, Acceleration
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Servo Pins:	     0,   1,   2,   3,   4,   5,   6,   -,   -
// Joint Name:	  head,necT,necB,eyeR,eyeL,armL,armR,eyebL,exbR,motL,motR
float curpos[] = { 248, 400, 140, 475, 270, 250, 290, 210, 307, 180, 180 };    // Current position (units)
float setpos[] = { 248, 400, 140, 475, 270, 250, 290, 210, 307, 0, 0 };        // Required position (units)
float curvel[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };                          // Current velocity (units/sec)
float maxvel[] = { 500, 400, 500, 2400, 2400, 600, 600, 400, 400, 250, 250 };  // Max Servo velocity (units/sec)
float accell[] = { 350, 300, 480, 1800, 1800, 500, 500, 500, 500, 800, 800 };  // Servo acceleration (units/sec^2)

boolean ledMode = true;

// -------------------------------------------------------------------
/// Initial setup
// -------------------------------------------------------------------

// -------------------------------------------------------------------
// Initialize WiFi
// -------------------------------------------------------------------

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("Connecting to WiFi .."));
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    if (ledMode == true) {
      ledMode = false;
#ifdef LED_PIN
      pwm.setPin(LED_PIN, 0);
#endif
    } else {
      ledMode = true;
#ifdef LED_PIN
      pwm.setPWM(LED_PIN, 0, 512);
#endif
    }

    delay(1000);
  }
  Serial.print(F("Current IP : "));
  Serial.println(WiFi.localIP());
  // send current IP to mail recipient
}


// Initialize FileSystem

void initFS() {
#ifdef USE_LITTLEFS
  if (!LittleFS.begin()) {
    Serial.println(F("An error has occurred while mounting LittleFS"));
  }
  Serial.println(F("LittleFS mounted successfully"));
  // get file list for testing purpose
  //File root = LittleFS.open("/");


#else
  if (!SPIFFS.begin(false, "/spiffs", 30)) {
    Serial.println(F("An error has occurred while mounting SPIFFS"));
  }
  Serial.println(F("SPIFFS mounted successfully"));
  // get file list for testing purpose

  //File root = SPIFFS.open("/");
#endif
  /*
  File file = root.openNextFile();
  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.name());
    file = root.openNextFile();
  }
*/
}


void setup() {
 // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println(F("--- Wall-E Control Sketch ---"));


  // Output Enable (EO) pin for the servo motors
  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_ENABLE_PIN, HIGH);

  // Communicate with servo shield (Analog servos run at ~60Hz)
  pwm.begin();
  pwm.setPWMFreq(60);

  // Turn off servo outputs
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    pwm.setPin(i, 0);
  }

  // Initialize DFPlayer
  mySoftwareSerial.begin(9600, SERIAL_8N1,DFPLAYER_RX_PIN , DFPLAYER_TX_PIN);  // speed, type, RX, TX
  if (!myDFPlayer.begin(mySoftwareSerial)) {         //Use softwareSerial to communicate with mp3.
    // If failed, no sound available : continue
    isSound = false;
    Serial.println(myDFPlayer.readType(), HEX);
    Serial.println(F("Error with DFPlayer."));
  } else {
    Serial.println(F("DFPlayer Mini online."));
    myDFPlayer.setTimeOut(500);  //Set serial communictaion time out 500ms

    //----Set volume----
    myDFPlayer.volume(25);    //Set volume value (0~30).
    myDFPlayer.volumeUp();    //Volume Up
    myDFPlayer.volumeDown();  //Volume Down
    myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
    myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
    myDFPlayer.play(1);  //Play the first mp3
  }
  
  initFS();



  randomSeed(analogRead(0));

  // Check if servo animation queue is working, and move servos to known starting positions
  if (queue.errors()) Serial.println(F("Error: Unable to allocate memory for servo animation queue"));

  // Soft start the servo motors
  Serial.println(F("Starting up the servo motors"));
  digitalWrite(SERVO_ENABLE_PIN, LOW);
  initWiFi();

  playAnimation(0);
  softStart(queue.pop(), 3500);

  // Motor pins
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);

// If an oLED is present, start it up
#ifdef OLED
  Serial.println(F("Starting up the display"));
  u8g2.begin();
  displayLevel(100);
#endif

  // Init Web server
  // Web Server Root URL
#ifdef USE_LITTLEFS
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.serveStatic("/", LittleFS, "/");
#else
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });
  server.serveStatic("/", SPIFFS, "/");
#endif
  // Request for the latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{123}";
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/servoControl", onServoControl);
  server.on("/motor", onMotor);
  server.on("/animate", onAnimate);
  server.on("/settings", onSettings);
  server.on("/audio", onAudio);

  // Start server
  server.begin();


  Serial.println(F("Startup complete; entering main loop"));
}



void doEvaluateSerial(char fs, String val) {
  firstChar = fs;
  serialLength = 0;
  for (uint8_t i = 0; i < val.length() && i < MAX_SERIAL_LENGTH - 1; i++) {
    serialBuffer[i] = val[i];
    serialLength++;
  }
  serialBuffer[serialLength] = 0;
  /*Serial.print(" fs = ");
    Serial.print(fs);
    Serial.print(" serialBuffer = ");
    Serial.print(serialBuffer);
    Serial.print(", ");
    */
  evaluateSerial();
}

void onServoControl(AsyncWebServerRequest *request) {
  String servo = "";
  String value = "";
  if (request->args() > 0) {
    if (request->hasArg("servo"))
      servo = request->arg("servo");
    if (request->hasArg("value"))
      value = request->arg("value");
    request->send(200, "application/json", "{ \"status\": \"OK\" }");
    doEvaluateSerial(servo[0], value);
  } else {
    request->send(200, "application/json", "{ \"status\": \"Error\",\"msg\":\"Unable to read post Data\" }");
  }
}

void onMotor(AsyncWebServerRequest *request) {
  if (request->args() > 0) {
    if (request->hasArg("stickX") && request->hasArg("stickY")) {
      request->send(200, "application/json", "{ \"status\": \"OK\" }");
      doEvaluateSerial('X', String(int32_t(request->arg("stickX").toFloat() * 100)));
      doEvaluateSerial('Y', String(int32_t(request->arg("stickY").toFloat() * 100)));
    }
  } else {
    request->send(200, "application/json", "{ \"status\": \"Error\",\"msg\":\"Unable to read post Data\" }");
  }
}

void onAnimate(AsyncWebServerRequest *request) {
  if (request->args() > 0) {
    if (request->hasArg("clip")) {
      request->send(200, "application/json", "{ \"status\": \"OK\" }");
      doEvaluateSerial('A', request->arg("clip"));
    }
  } else {
    request->send(200, "application/json", "{ \"status\": \"Error\",\"msg\":\"Unable to read post Data\" }");
  }
}

void onSettings(AsyncWebServerRequest *request) {
  String thing = "";
  String value = "";
  if (request->args() > 0) {
    if (request->hasArg("type")) {
      thing = request->arg("type");
      value = request->arg("value");
    }
    if (thing == "motorOff") {
      doEvaluateSerial('O', value);
    } else if (thing == "steerOff") {
      doEvaluateSerial('S', value);
    } else if (thing == "animeMode") {
      doEvaluateSerial('M', value);
    }

    request->send(200, "application/json", "{ \"status\": \"OK\" }");


  } else {
    request->send(200, "application/json", "{ \"status\": \"Error\",\"msg\":\"Unable to read post Data\" }");
  }
}

void onAudio(AsyncWebServerRequest *request) {
  if (request->args() > 0) {
    if (request->hasArg("clip")) {
      request->send(200, "application/json", "{ \"status\": \"OK\" }");
      doEvaluateSerial('!', request->arg("clip"));
    }
  } else {
    request->send(200, "application/json", "{ \"status\": \"Error\",\"msg\":\"Unable to read post Data\" }");
  }
}


// -------------------------------------------------------------------
/// Read input from serial port
///
/// This function reads incoming characters in the serial port
/// and inserts them into a buffer to be processed later.
// -------------------------------------------------------------------

void readSerial() {
  // Read incoming byte
  char inchar = Serial.read();

  // If the string has ended, evaluate the serial buffer
  if (inchar == '\n' || inchar == '\r') {

    if (serialLength > 0) evaluateSerial();
    serialBuffer[0] = 0;
    serialLength = 0;

    // Otherwise add to the character to the buffer
  } else {
    if (serialLength == 0) firstChar = inchar;
    else {
      serialBuffer[serialLength - 1] = inchar;
      serialBuffer[serialLength] = 0;
    }
    serialLength++;

    // To prevent overflows, evalute the buffer if it is full
    if (serialLength == MAX_SERIAL_LENGTH) {
      evaluateSerial();
      serialBuffer[0] = 0;
      serialLength = 0;
    }
  }
}



// -------------------------------------------------------------------
/// Evaluate input from serial port
///
/// Parse the received serial message which is stored in
/// the "serialBuffer" filled by the "readSerial()" function
// -------------------------------------------------------------------

void evaluateSerial() {
  // Evaluate integer number in the serial buffer
  int number = atoi(serialBuffer);

  Serial.print(firstChar);
  Serial.println(number);


  // Motor Inputs and Offsets
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if (firstChar == 'X' && number >= -100 && number <= 100) turnValue = int(number * 2.55);       // Left/right control
  else if (firstChar == 'Y' && number >= -100 && number <= 100) moveValue = int(number * 2.55);  // Forward/reverse control
  else if (firstChar == 'S' && number >= -100 && number <= 100) turnOffset = number;             // Steering offset
  else if (firstChar == 'O' && number >= 0 && number <= 250) motorDeadzone = int(number);        // Motor deadzone offset


  // Animations
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  else if (firstChar == 'A') playAnimation(number);


  // Autonomous servo mode
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  else if (firstChar == 'M' && number == 0) autoMode = false;
  else if (firstChar == 'M' && number == 1) autoMode = true;


  // Manual servo control
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  else if (firstChar == 'L' && number >= 0 && number <= 100) {  // Move left arm
    autoMode = false;
    queue.clear();
    setpos[5] = int(number * 0.01 * (preset[5][1] - preset[5][0]) + preset[5][0]);
  } else if (firstChar == 'R' && number >= 0 && number <= 100) {  // Move right arm
    autoMode = false;
    queue.clear();
    setpos[6] = int(number * 0.01 * (preset[6][1] - preset[6][0]) + preset[6][0]);
  } else if (firstChar == 'B' && number >= 0 && number <= 100) {  // Move neck bottom
    autoMode = false;
    queue.clear();
    setpos[2] = int(number * 0.01 * (preset[2][1] - preset[2][0]) + preset[2][0]);
  } else if (firstChar == 'T' && number >= 0 && number <= 100) {  // Move neck top
    autoMode = false;
    queue.clear();
    setpos[1] = int(number * 0.01 * (preset[1][1] - preset[1][0]) + preset[1][0]);
  } else if (firstChar == 'G' && number >= 0 && number <= 100) {  // Move head rotation
    autoMode = false;
    queue.clear();
    setpos[0] = int(number * 0.01 * (preset[0][1] - preset[0][0]) + preset[0][0]);
  } else if (firstChar == 'E' && number >= 0 && number <= 100) {  // Move eye left
    autoMode = false;
    queue.clear();
    setpos[4] = int(number * 0.01 * (preset[4][1] - preset[4][0]) + preset[4][0]);
  } else if (firstChar == 'U' && number >= 0 && number <= 100) {  // Move eye right
    autoMode = false;
    queue.clear();
    setpos[3] = int(number * 0.01 * (preset[3][1] - preset[3][0]) + preset[3][0]);
  } else if (firstChar == 'C' && number >= 0 && number <= 100) {  // Move eyebrow left
    autoMode = false;
    queue.clear();
    setpos[8] = int(number * 0.01 * (preset[8][1] - preset[8][0]) + preset[8][0]);
  } else if (firstChar == 'V' && number >= 0 && number <= 100) {  // Move eyebrow right
    autoMode = false;
    queue.clear();
    setpos[7] = int(number * 0.01 * (preset[7][1] - preset[7][0]) + preset[7][0]);
  }


  // Manual Movements with WASD
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  else if (firstChar == 'w') {  // Forward movement
    moveValue = pwmspeed;
    turnValue = 0;
    setpos[0] = (preset[0][1] + preset[0][0]) / 2;
  } else if (firstChar == 'q') {  // Stop movement
    moveValue = 0;
    turnValue = 0;
    setpos[0] = (preset[0][1] + preset[0][0]) / 2;
  } else if (firstChar == 's') {  // Backward movement
    moveValue = -pwmspeed;
    turnValue = 0;
    setpos[0] = (preset[0][1] + preset[0][0]) / 2;
  } else if (firstChar == 'a') {  // Drive & look left
    moveValue = 0;
    turnValue = -pwmspeed;
    setpos[0] = preset[0][0];
  } else if (firstChar == 'd') {  // Drive & look right
    moveValue = 0;
    turnValue = pwmspeed;
    setpos[0] = preset[0][1];
  }


  // Manual Eye Movements
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  else if (firstChar == 'j') {  // Left head tilt
    setpos[4] = preset[4][0];
    setpos[3] = preset[3][1];
  } else if (firstChar == 'l') {  // Right head tilt
    setpos[4] = preset[4][1];
    setpos[3] = preset[3][0];
  } else if (firstChar == 'i') {  // Sad head
    setpos[4] = preset[4][0];
    setpos[3] = preset[3][0];
  } else if (firstChar == 'k') {  // Neutral head
    setpos[4] = int(0.4 * (preset[4][1] - preset[4][0]) + preset[4][0]);
    setpos[3] = int(0.4 * (preset[3][1] - preset[3][0]) + preset[3][0]);
  }


  // Head movement
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  else if (firstChar == 'f') {  // Head up
    setpos[1] = preset[1][1];
    setpos[2] = preset[2][1]; //(preset[2][1] + preset[2][0]) / 2;
  } else if (firstChar == 'g') {  // Head forward
    setpos[1] = preset[1][1];
    setpos[2] = preset[2][0];
  } else if (firstChar == 'h') {  // Head down
    setpos[1] = preset[1][0];
    setpos[2] = preset[2][0];
  }


  // Arm Movements
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  else if (firstChar == 'b') {  // Left arm low, right arm high
    setpos[5] = preset[5][0];
    setpos[6] = preset[6][1];
  } else if (firstChar == 'n') {  // Both arms neutral
    setpos[5] = (preset[5][0] + preset[5][1]) / 2;
    setpos[6] = (preset[6][0] + preset[6][1]) / 2;
  } else if (firstChar == 'm') {  // Left arm high, right arm low
    setpos[5] = preset[5][1];
    setpos[6] = preset[6][0];
  }

  else if (firstChar == '!') {  // Play sound
    soundPlaying = true;
    myDFPlayer.play(number + 1);  //Play the corresponding mp3
  } else if (firstChar == 'v') {  // Set volume
    myDFPlayer.volume(number);    //Set the volume to the value
  }
}


// -------------------------------------------------------------------
/// Sequence and generate animations
// -------------------------------------------------------------------

void manageAnimations() {
  // If we are running an animation
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if ((queue.size() > 0) && (animeTimer <= millis())) {
    // Set the next waypoint time
    animation_t newValues = queue.pop();
    animeTimer = millis() + newValues.timer;

    // Set all the joint positions
    for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
      // Scale the positions using the servo calibration values
      setpos[i] = int(newValues.servos[i] * 0.01 * (preset[i][1] - preset[i][0]) + preset[i][0]);
    }


    // If we are in autonomous mode and no movements are queued, generate random movements
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  } else if (autoMode && queue.empty() && (animeTimer <= millis())) {

    // For each of the servos
    for (int i = 0; i < NUMBER_OF_SERVOS; i++) {

      // Randomly determine whether or not to update the servo
      if (random(2) == 1) {

        // For most of the servo motors
        if (i == 0 || i == 1 || i == 2 || i >= 5) {

          // Randomly determine the new position
          unsigned int min = preset[i][0];
          unsigned int max = preset[i][1];
          if (min > max) {
            min = max;
            max = preset[i][0];
          }

          setpos[i] = random(min, max + 1);

          // Since the eyes should work together, only look at one of them
        } else if (i == 3) {

          int midPos1 = int((preset[i][1] - preset[i][0]) * 0.4 + preset[i][0]);
          int midPos2 = int((preset[i + 1][1] - preset[i + 1][0]) * 0.4 + preset[i + 1][0]);

          // Determine which type of eye movement to do
          // Both eye move downwards
          if (random(2) == 1) {
            setpos[i] = random(midPos1, preset[i][0]);
            float multiplier = (setpos[i] - midPos1) / float(preset[i][0] - midPos1);
            setpos[i + 1] = ((1 - multiplier) * (midPos2 - preset[i + 1][0])) + preset[i + 1][0];

            // Both eyes move in opposite directions
          } else {
            setpos[i] = random(midPos1, preset[i][0]);
            float multiplier = (setpos[i] - preset[i][1]) / float(preset[i][0] - preset[i][1]);
            setpos[i + 1] = (multiplier * (preset[i + 1][1] - preset[i + 1][0])) + preset[i + 1][0];
          }
        }
      }
      //Serial.print(setpos[i]);
      //Serial.print(",");
    }
    //Serial.println();

    // Finally, figure out the amount of time until the next movement should be done
    animeTimer = millis() + random(500, 3000);
  }
}



// -------------------------------------------------------------------
/// Manage the movement of the servo motors
///
/// @param  dt  Time in milliseconds since function was last called
///
/// This function uses the formulae:
///   (s = position, v = velocity, a = acceleration, t = time)
///   s = v^2 / (2*a)  <- to figure out whether to start slowing down
///   v = v + a*t      <- to calculate new servo velocity
///   s = s + v*t      <- to calculate new servo position
// -------------------------------------------------------------------

void manageServos(float dt) {
  bool moving = false;

  // For each of the servo motors
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {

    float posError = setpos[i] - curpos[i];

    // If position error is above the threshold
    if (abs(posError) > CONTROLLER_THRESHOLD && (setpos[i] != -1)) {

      digitalWrite(SERVO_ENABLE_PIN, LOW);
      moving = true;

      // Determine motion direction
      bool dir = true;
      if (posError < 0) dir = false;

      // Determine whether to accelerate or decelerate
      float acceleration = accell[i];
      if ((curvel[i] * curvel[i] / (2 * accell[i])) > abs(posError)) acceleration = -accell[i];

      // Update the current velocity
      if (dir) curvel[i] += acceleration * dt / 1000.0;
      else curvel[i] -= acceleration * dt / 1000.0;

      // Limit Velocity
      if (curvel[i] > maxvel[i]) curvel[i] = maxvel[i];
      if (curvel[i] < -maxvel[i]) curvel[i] = -maxvel[i];

      float dP = curvel[i] * dt / 1000.0;

      if (abs(dP) < abs(posError)) curpos[i] += dP;
      else curpos[i] = setpos[i];

      pwm.setPWM(i, 0, curpos[i]);

    } else {
      curvel[i] = 0;
    }
  }

  // Disable servos if robot is not moving
  // This helps prevents the motors from overheating
  if (moving) motorTimer = millis();
  else if (millis() - motorTimer >= SERVO_OFF_TIME) {
    //digitalWrite(SERVO_ENABLE_PIN, HIGH);
    for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
      pwm.setPin(i, 0);
    }
  }
}



// -------------------------------------------------------------------
/// Servo "Soft Start" function
///
/// This function tries to start the servos up servo gently,
/// reducing the sudden jerking motion which usually occurs
/// when the motors power up for the first time.
///
/// @param  targetPos  The target position of the servos after startup
/// @param  timeMs     Time in milliseconds in which soft start should complete
// -------------------------------------------------------------------

void softStart(animation_t targetPos, int timeMs) {
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    if (targetPos.servos[i] >= 0) {
      curpos[i] = int(targetPos.servos[i] * 0.01 * (preset[i][1] - preset[i][0]) + preset[i][0]);

      unsigned long endTime = millis() + timeMs / NUMBER_OF_SERVOS;

      while (millis() < endTime) {
        pwm.setPWM(i, 0, curpos[i]);
        delay(10);
        pwm.setPin(i, 0);
        delay(50);
      }
      pwm.setPWM(i, 0, curpos[i]);
      setpos[i] = curpos[i];
    }
  }
}



// -------------------------------------------------------------------
/// Manage the movement of the main motors
///
/// @param  dt  Time in milliseconds since function was last called
// -------------------------------------------------------------------

void manageMotors(float dt) {
  // Update Main Motor Values
  setpos[NUMBER_OF_SERVOS] = moveValue - turnValue;
  setpos[NUMBER_OF_SERVOS + 1] = moveValue + turnValue;

  // Apply turn offset (motor trim) only when motors are active
  if (setpos[NUMBER_OF_SERVOS] != 0) setpos[NUMBER_OF_SERVOS] -= turnOffset;
  if (setpos[NUMBER_OF_SERVOS + 1] != 0) setpos[NUMBER_OF_SERVOS + 1] += turnOffset;

  for (int i = NUMBER_OF_SERVOS; i < NUMBER_OF_SERVOS + 2; i++) {

    float velError = setpos[i] - curvel[i];

    // If velocity error is above the threshold
    if (abs(velError) > CONTROLLER_THRESHOLD && (setpos[i] != -1)) {

      // Determine whether to accelerate or decelerate
      float acceleration = accell[i];
      if (setpos[i] < curvel[i] && curvel[i] >= 0) acceleration = -accell[i];
      else if (setpos[i] < curvel[i] && curvel[i] < 0) acceleration = -accell[i];
      else if (setpos[i] > curvel[i] && curvel[i] < 0) acceleration = accell[i];

      // Update the current velocity
      float dV = acceleration * dt / 1000.0;
      if (abs(dV) < abs(velError)) curvel[i] += dV;
      else curvel[i] = setpos[i];
    } else {
      curvel[i] = setpos[i];
    }

    // Apply deadzone offset
    if (curvel[i] > 0) curvel[i] += motorDeadzone;
    else if (curvel[i] < 0) curvel[i] -= motorDeadzone;

    // Limit Velocity
    if (curvel[i] > maxvel[i]) curvel[i] = maxvel[i];
    if (curvel[i] < -maxvel[i]) curvel[i] = -maxvel[i];
  }

  // Update motor speeds
  motorL.setSpeed(curvel[NUMBER_OF_SERVOS]);
  motorR.setSpeed(curvel[NUMBER_OF_SERVOS + 1]);
#ifdef LED_PIN
  pwm.setPWM(LED_PIN, 0, abs(curvel[NUMBER_OF_SERVOS]) * 2);
#endif
  /* // debugging
    if (curvel[NUMBER_OF_SERVOS] != oldCurvel) {
      Serial.print("Motor values : ");
      Serial.print(curvel[NUMBER_OF_SERVOS]);
      Serial.print(" : ");
      Serial.println(curvel[NUMBER_OF_SERVOS + 1]);
      oldCurvel = curvel[NUMBER_OF_SERVOS];
    }*/
}



// -------------------------------------------------------------------
/// Battery level detection
// -------------------------------------------------------------------

#ifdef BAT_L
void checkBatteryLevel() {
  // Read the analogue pin and calculate battery voltage
  float voltage = analogRead(BATTERY_LEVEL_PIN) * 5 / 1024.0;
  voltage = voltage / DIVIDER_SCALING_FACTOR;
  int percentage = int(100 * (voltage - BATTERY_MIN_VOLTAGE) / float(BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE));

// Update the oLed Display if installed
#ifdef OLED
  displayLevel(percentage);
#endif

  // Send the percentage via serial
  Serial.print(F("Battery_"));
  Serial.println(percentage);
}
#endif


// -------------------------------------------------------------------
/// Main program loop
// -------------------------------------------------------------------

void loop() {

  if (soundPlaying) {
    // Has it completed?
    //      DacAudio.FillBuffer();
  }

  // Read any new serial messages
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if (Serial.available() > 0) {
    readSerial();
  }


  // Load or generate new animations
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  manageAnimations();


  // Move Servos and wheels at regular time intervals
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if (millis() - updateTimer >= SERVO_UPDATE_TIME) {
    updateTimer = millis();

    unsigned long newTime = micros();
    float dt = (newTime - lastTime) / 1000.0;
    lastTime = newTime;

    manageServos(dt);
    manageMotors(dt);
  }

  // Update robot status
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if (millis() - statusTimer >= STATUS_CHECK_TIME) {
    statusTimer = millis();
    //      pwm.setPWM(LED_PIN, 0, 400);

#ifdef BAT_L
    checkBatteryLevel();
#endif
  }
}
