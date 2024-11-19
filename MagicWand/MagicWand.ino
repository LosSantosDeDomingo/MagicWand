///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Publisher(s): Jose Caraballo, Adrian Suarez
// School: Florida Atlantic University
// Professor: Dr. Alhalabi
// GitHub Repository Link: https://github.com/LosSantosDeDomingo/MagicWand
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Micro-Controller: NodeMCU ESP-32S
// Arduino IDE Board: NodeMCU-32S
// Board Manager: esp32 by Espressif Systems 2.0.17
// Arduino IDE Port: 12
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Purpose: 
  This program was developed to create a security system that can be locked or unlocked through the use of the
  accelerometer and gyroscope from a cellphone. The motions from the phone will follow a 0-6 numbering system.
  Additionally, the program will allow for two programs to be stored within the ESP-32S.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Immediate improvements for current version:
  - Test Code

  Possible improvements:
  - Voice status
  - LCD display
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Date Created: 10.23.2024
// Last Revision: 11.18.2024
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preprocessor Directives
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <ESP32Servo.h> // Version: 3.0.5
#include <DabbleESP32.h> // Version: 1.5.1
#include <Preferences.h>
#include <math.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define Directives
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Baud Rate Directive
#define BAUD_RATE 115200

// Dabble App Directives
#define CUSTOM_SETTINGS
#define INCLUDE_SENSOR_MODULE

// Constant Directives
#define GRAVITY 9.81
#define PI 3.14
#define BASELINE 0

// Coefficient Directives
#define MOVEMENT_COEFFICIENT 0.8
#define GRAVITY_COEFFICIENT 1.2

// Logic Directives
#define LOGICAL_HIGH 1

// Conversion Directives
#define ANGLE 180.0

// MISC. Directives
#define SQUARED 2
#define INCREMENT 1
#define MOVEMENT_THRESHOLD 1.0
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ENUM
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum phoneGesture {
  downToUp = 1,
  upToDown = 2,
  leftToRight = 3,
  rightToLeft = 4,
  centerToBack = 5,
  centerToForward = 6,
  noSignificantMovement = -1,
} gesture = noSignificantMovement;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Objects
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Preferences security;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Run System Function
void runGestureSystem();

// System Connection Functions
void confirmPortConnection();
void confirmDabbleConnection();

// Interrupt Functions
void setupInterrupt();
void IRAM_ATTR handleButtonPress();
void initializeSecuritySystem();

// Peripherals Functions
void setupPeripherals(const int motionSensorPin, const int pushButtonPin, const int sinusoidLockPin);
void initializeMotionSensor(const int motionSensorPin);
void initializePushButton(const int pushButtonPin);
void initializeLockMechanism(const int sinusoidLockPin);
void openLockMechanism();

// Accelerometer Functions
void setupAccelerometer(float* initialAccelXAxis, float* initialAccelYAxis, float* initialAccelZAxis);
void initializeOrientation(float initialAccelXAxis, float initialAccelYAxis, float initialAccelZAxis);
void processAccelData(float* accelerometerXAxis, float* accelerometerYAxis, float* accelerometerZAxis);
float filterAccelXAxisData(float accelXAxis);
float filterAccelYAxisData(float accelYAxis);
float filterAccelZAxisData(float accelZAxis);
void printAccelerometerData(float accelXAxis, float accelYAxis, float accelZAxis);

// Password Functions
void createSecurityConnection();
void createPassword(phoneGesture detectedGesture);
void detectAccelGesture(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis);
void gestureToPin(phoneGesture detectedGesture);
void updatePinCode(String pinCode);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripheral Locations
const int pushButtonPin = 39;
const int motionSensorPin = 2;
const int sinusoidLockPin = 15;

// Password Variables
String pinReset = "";
String firstPinCode = "";
String secondPinCode = "";
const int passwordDigits = 6;

// Accelerometer Variables
// Initial Sensor Data
float initialAccelXAxis = 0;
float initialAccelYAxis = 0;
float initialAccelZAxis = 0;

// Filtered Sensor Data
float accelerometerXAxis = 0;
float accelerometerYAxis = 0;
float accelerometerZAxis = 0;

// Gradient Sensor Data
float deltaAccelXAxis = 0;
float deltaAccelYAxis = 0;
float deltaAccelZAxis = 0;

// Timer Variables
// Real Time Clock
unsigned long currentTime = 0;
unsigned long startTime = 0;

// Delays
unsigned long dabbleDelay = 15000;
unsigned long buttonDelay = 500;
unsigned long debounceDelay = 200;
unsigned long sequenceDelay = 1000;
unsigned long filterDelay = 20;
unsigned long waitDelay = 20;
unsigned long lockDelay = 20000;

// Flag Variables
bool dabbleStatus = false;
bool passwordModeActive = false;
bool firstPinSet = false;
bool systemArmed = false;

// Counter Variables
int gestureCount = 0;
volatile int buttonPressCount = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Code Drivers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  confirmPortConnection();
  setupPeripherals(motionSensorPin, pushButtonPin, sinusoidLockPin);
  setupInterrupt();
  initializeSecuritySystem();
  createSecurityConnection();
  confirmDabbleConnection();
  setupAccelerometer(&initialAccelXAxis, &initialAccelYAxis, &initialAccelZAxis);
}

void loop() {
  Dabble.processInput();
  runGestureSystem();
  security.end();
  delay(waitDelay);
  esp_deep_sleep_start(); // Moved deep sleep to the correct control flow area
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Function Definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Run System Function
void runGestureSystem(){
  currentTime = millis();
  if (digitalRead(pushButtonPin) == HIGH && (currentTime - startTime) > buttonDelay) {
    passwordModeActive = !passwordModeActive;
    startTime = currentTime;
    Serial.println(passwordModeActive ? "Password mode activated..." : "Password mode deactivated...");
    if (passwordModeActive) {
      firstPinCode = pinReset;
      gesture = noSignificantMovement;
    }
  }

  if (passwordModeActive) {
    processAccelData(&accelerometerXAxis, &accelerometerYAxis, &accelerometerZAxis);
    detectAccelGesture(accelerometerXAxis, accelerometerYAxis, accelerometerZAxis);
    createPassword(gesture);
  }
}

// System Connection Functions
void confirmPortConnection(){
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  delay(waitDelay);
  Serial.println("Serial Port Connection Successful!");
}

void confirmDabbleConnection(){
  Dabble.begin("MagicWand");
  Serial.println("Waiting to connect to Dabble...");
  Dabble.delay(dabbleDelay); // 15 second delay time
  Serial.println("\nDabble is Connected.");
}

// Interrupt Functions
void setupInterrupt(){
  attachInterrupt(digitalPinToInterrupt(pushButtonPin), handleButtonPress, FALLING);
}

void IRAM_ATTR handleButtonPress() {
  currentTime = millis();
  if (currentTime - startTime > debounceDelay) {
      buttonPressCount++;
      startTime = currentTime;
  }
}

void initializeSecuritySystem() {
  // Enable wakeup by the motion sensor
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(motionSensorPin), LOGICAL_HIGH);

  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woken up by motion!");
  } 
  else{
    Serial.println("System started normally or reset.");
  }

  security.begin("securitySystem", false);
  systemArmed = security.getBool("isArmed", true);
  Serial.print("System is ");
  Serial.println(systemArmed ? "armed." : "disarmed.");
}

// Peripherals Functions
void setupPeripherals(const int motionSensorPin, const int pushButtonPin, const int sinusoidLockPin){
  initializeMotionSensor(motionSensorPin);
  initializePushButton(pushButtonPin);
  initializeLockMechanism(sinusoidLockPin);
}
void initializeMotionSensor(const int motionSensorPin){
  pinMode(motionSensorPin, INPUT);
}

void initializePushButton(const int pushButtonPin){
  pinMode(pushButtonPin, INPUT);
}

void initializeLockMechanism(const int sinusoidLockPin){
 pinMode(sinusoidLockPin, OUTPUT);
 digitalWrite(sinusoidLockPin, LOW);
}

void openLockMechanism(){
  digitalWrite(sinusoidLockPin, HIGH);
  Serial.println("Door is unlocked...");
  delay(lockDelay);
  digitalWrite(sinusoidLockPin, LOW);
  Serial.println("Door is locked...");
}

// Accelerometer Functions
void setupAccelerometer(float* initialAccelXAxis, float* initialAccelYAxis, float* initialAccelZAxis){
  // Grab Initial Position
  *initialAccelXAxis = filterAccelXAxisData(*initialAccelXAxis);
  *initialAccelYAxis = filterAccelYAxisData(*initialAccelYAxis);
  *initialAccelZAxis = filterAccelZAxisData(*initialAccelZAxis);
  initializeOrientation(*initialAccelXAxis, *initialAccelYAxis, *initialAccelZAxis);
}

void initializeOrientation(float initialAccelXAxis, float initialAccelYAxis, float initialAccelZAxis) {
  float movementOrienThreshold = MOVEMENT_COEFFICIENT * GRAVITY;
  float gravityThreshold = GRAVITY_COEFFICIENT * GRAVITY;

  // Determine the Orientation of the Phone
  if (fabs(initialAccelZAxis) > movementOrienThreshold && fabs(initialAccelZAxis) < gravityThreshold) {
    if (initialAccelZAxis > BASELINE ? Serial.println("Flat, screen up") : Serial.println("Flat, screen down"));
  }
  else if (fabs(initialAccelXAxis) > movementOrienThreshold && fabs(initialAccelXAxis) < gravityThreshold) {
    if (initialAccelXAxis > BASELINE ? Serial.println("Landscape, right side down") : Serial.println("Landscape, left side down"));
  } 
  else if (fabs(initialAccelYAxis) > movementOrienThreshold and fabs(initialAccelYAxis) < gravityThreshold) {
    if (initialAccelYAxis > BASELINE ? Serial.println("Portrait, bottom down") : Serial.println("Portrait, top down"));
  }
  else {
    Serial.println("Uncertain or in motion");
  }
}

void processAccelData(float* accelerometerXAxis, float* accelerometerYAxis, float* accelerometerZAxis){
  // Filter Accel Data
  *accelerometerXAxis = filterAccelXAxisData(*accelerometerXAxis);
  *accelerometerYAxis = filterAccelYAxisData(*accelerometerYAxis);
  *accelerometerZAxis = filterAccelZAxisData(*accelerometerZAxis);

  // Debugging Outputs
  printAccelerometerData(*accelerometerXAxis, *accelerometerYAxis, *accelerometerZAxis);
}

float filterAccelXAxisData(float accelXAxis){
  int accelXAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  currentTime = millis();
  while((currentTime - startTime) < filterDelay){
    accelXAxis += Sensor.getAccelerometerXaxis();
    accelXAxisCounter += INCREMENT;
    startTime = currentTime;
  }

  // Return Filtered Data
  return accelXAxis / (float)accelXAxisCounter;
}

float filterAccelYAxisData(float accelYAxis){
  int accelYAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  currentTime = millis();
  while((currentTime - startTime) < filterDelay){
    accelYAxis += Sensor.getAccelerometerYaxis();
    accelYAxisCounter += INCREMENT;
    startTime = currentTime;
  }

  // Return Filtered Data
  return accelYAxis / (float)accelYAxisCounter;
}

float filterAccelZAxisData(float accelZAxis){
  int accelZAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  currentTime = millis();
  while((currentTime - startTime) < filterDelay){
    accelZAxis += Sensor.getAccelerometerZaxis();
    accelZAxisCounter += INCREMENT;
    startTime = currentTime;
  }

  // Return Filtered Data
  return accelZAxis / (float)accelZAxisCounter;
}

void printAccelerometerData(float accelXAxis, float accelYAxis, float accelZAxis){
  Serial.println("---------------------------");
  Serial.println("Accelerometer Data");
  Serial.println("---------------------------");
  Serial.print("X-Axis:"); Serial.print(accelXAxis); Serial.print(" Y-Axis:"); Serial.print(accelYAxis); Serial.print(" Z-Axis:"); Serial.println(accelZAxis); 
  // Serial.print("Accel Magnitude: "); Serial.println(accelerometerMagnitude);
  Serial.println("---------------------------");
}

// Password Functions
void createSecurityConnection(){
  security.begin("passwordBank", false);
  if(!security.getString("firstPinCode", "").length()){ // Check if empty
    security.putString("firstPinCode", "");
    security.putString("secondPinCode", "");
  }
}

void createPassword(phoneGesture detectedGesture){
  if(detectedGesture != noSignificantMovement){
    gestureToPin(detectedGesture);
  }
}

void detectAccelGesture(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis) {
  // Calculate Change in Positions
  deltaAccelXAxis = filteredAccelXAxis - initialAccelXAxis;
  deltaAccelYAxis = filteredAccelYAxis - initialAccelYAxis;
  deltaAccelZAxis = filteredAccelZAxis - initialAccelZAxis;

  // Update the Initial Position
  initialAccelXAxis = filteredAccelXAxis;  
  initialAccelYAxis = filteredAccelYAxis; 
  initialAccelZAxis = filteredAccelZAxis; 

  // Determine Phone Gesture
  if (fabs(deltaAccelYAxis) > MOVEMENT_THRESHOLD){
    gesture = (deltaAccelYAxis > BASELINE ? downToUp : upToDown);
  }
  else if (fabs(deltaAccelXAxis) > MOVEMENT_THRESHOLD){
    gesture = (deltaAccelXAxis > BASELINE ? leftToRight : rightToLeft);
  }
  else if (fabs(deltaAccelZAxis) > MOVEMENT_THRESHOLD){
    gesture = (deltaAccelZAxis > BASELINE ? centerToBack : centerToForward);
  } 
  else { 
    gesture = noSignificantMovement;
  }
}

void gestureToPin(phoneGesture detectedGesture){
  if(gestureCount < passwordDigits){
    firstPinCode += String(static_cast<int>(detectedGesture));
    gestureCount++;
    Serial.println("Gesture Detected: " + String(static_cast<int>(detectedGesture)));
    
    if(gestureCount == passwordDigits) {
      if(firstPinCode == security.getString("firstPinCode", "") || secondPinCode == security.getString("secondPinCode", "")){
        Serial.println("Valid password entered.");
        openLockMechanism();
      } 
      else {
        Serial.println("Invalid password.");
      }

      firstPinCode = pinReset;
      gestureCount = BASELINE;
      passwordModeActive = false;
    }
  }
}

void updatePinCode(String pinCode){
  if(!firstPinSet) {
    security.putString("firstPinCode", pinCode);
    firstPinSet = true;
    Serial.println("First Pin Code Saved: " + pinCode);
  } 
  else if(security.getString("secondPinCode", "").isEmpty()) {
    security.putString("secondPinCode", pinCode);
    Serial.println("Second Pin Code Saved: " + pinCode);
  } 
  else {
    Serial.println("Both Pin Codes are already set. No update performed.");
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
