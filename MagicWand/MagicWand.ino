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
  accelerometer from a mobile device. The motions from the phone will follow a 1-6 numbering system.
  Additionally, the program will allow for a password to be stored within the ESP-32S.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Immediate improvements for current version:
  - Fix Password Setup Mode Reset
  - Fix Gesture Readings

  Possible improvements:
  - LCD display
  - Machine Learning Training
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Date Created: 10.23.2024
// Last Revision: 12.6.2024
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preprocessor Directives
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ESP32Servo.h>   // Version: 3.0.5
#include <DabbleESP32.h>  // Version: 1.5.1
#include <Preferences.h>
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
#define BASELINE 0

// Coefficient Directives
#define MOVEMENT_COEFFICIENT 0.8
#define GRAVITY_COEFFICIENT 1.2

// Logic Directives
#define LOGICAL_HIGH 1

// MISC. Directives
#define INCREMENT 1
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
// System Connection Functions
void confirmPortConnection();
void confirmDabbleConnection();

// Button Functions
void setupInterrupt();
void IRAM_ATTR handleButtonPress();
void IRAM_ATTR handleResetButtonPress();
void manageButtonPresses();
void setupMotionSensor();
void IRAM_ATTR handleMotionInterrupt();

// Peripherals Functions
void setupPeripherals(const int motionSensor, const int pushButton, const int sinusoidLock);
void openLockMechanism();

// Accelerometer Functions
void setupAccelerometer(float* initialAccelXAxis, float* initialAccelYAxis, float* initialAccelZAxis);
void initializeOrientation(float initialAccelXAxis, float initialAccelYAxis, float initialAccelZAxis);
void processAccelData(float* accelerometerXAxis, float* accelerometerYAxis, float* accelerometerZAxis);
float filterAccelXAxisData(float accelXAxis);
float filterAccelYAxisData(float accelYAxis);
float filterAccelZAxisData(float accelZAxis);
void printAccelerometerData(float accelXAxis, float accelYAxis, float accelZAxis);

// Gesture Functions
void detectAccelGesture(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis);
void addGestureToBuffer(phoneGesture detectedGesture);
void clearGestureBuffer();

// Password Functions
void createSecurityConnection();
void verifyPassword();
void setupPassword();
void updatePinCode(String& pinCode, const String& key);
void resetPinCode(const String& key);

// Sleep Mode Functions
void sleepMode();
void initializeSecuritySystem();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripheral Locations
const int pushButton = 32;
const int resetButton = 33;
const int motionSensor = 2;
const int sinusoidLock = 15;

// Gesture Variables
phoneGesture lastDetectedGesture = noSignificantMovement;
const int gestureBufferSize = 6;
phoneGesture gestureBuffer[gestureBufferSize] = {noSignificantMovement};
int gestureBufferIndex = 0;

// Password Variables
String currentPinCode = "";
String firstPinCode = "";
const String factoryPin = "434343";
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

// Delta Data
float deltaAccelXAxis;
float deltaAccelYAxis;
float deltaAccelZAxis;

// Threshold
float MOVEMENT_THRESHOLD = 1.2;

// Time Variables
const unsigned long dabbleDelay = 15000;    // 15 seconds
const unsigned long debounceDelay = 1000;   // 1 second
const unsigned long filterDelay = 20;       // 20 milliseconds
const unsigned long lockDelay = 20000;      // 20 seconds
const unsigned long motionTimeout = 10000;  // 10 seconds

// Flag Variables
bool passwordModeActive = false;
bool setupPasswordMode = false;
bool motionDetected = false;
volatile bool isUnlockButtonPressed = false;
volatile bool isResetButtonPressed = false;
bool isProcessAccelActive = false;

// Counter Variables
volatile int buttonPressCount = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Code Drivers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  confirmPortConnection();
  setupPeripherals(motionSensor, pushButton, sinusoidLock);
  setupInterrupt();
  setupMotionSensor();         // Initialize the motion sensor
  initializeSecuritySystem();  // Includes wake-up reason and preference setup
  createSecurityConnection();  // Checks and informs about the security PIN status
  confirmDabbleConnection();   // Establish connection with Dabble
  setupAccelerometer(&initialAccelXAxis, &initialAccelYAxis, &initialAccelZAxis);
}

void loop() {
  // Local Timer Variables
  static unsigned long lastCheckTime = 0;
  const unsigned long checkInterval = 1000;

  // Process Dabble Inputs
  Dabble.processInput();

  // Handle Button Presses
  if (isUnlockButtonPressed || isResetButtonPressed) {
    manageButtonPresses();
  }

  // Process Accelerometer Data
  if (isProcessAccelActive) {
    handleAccelerometerProcessing();
  }

  // Handle Setup Mode
  if (setupPasswordMode) {
    setupPassword();
  }

    // Handle Verification Mode
  if (passwordModeActive) {
    verifyPassword();
  }

  // Check for inactivity and enter sleep mode if needed
  sleepMode();

  // Handle Dabble Disconnection Check
  unsigned long currentTime = millis();
  if (!Dabble.isAppConnected() && (currentTime - lastCheckTime >= checkInterval)) {
    lastCheckTime = currentTime;
    Serial.println("Dabble is not connected. Checking again...");
    sleepMode();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Function Definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System Connection Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void confirmPortConnection() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }
  Serial.println("Serial Port Connection Successful!");
}

void confirmDabbleConnection() {
  Dabble.begin("MagicWand");
  Serial.println("Waiting to connect to Dabble...");
  Dabble.delay(dabbleDelay);
  Serial.println("\nDabble is Connected.");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setupInterrupt() {
  attachInterrupt(digitalPinToInterrupt(pushButton), handleButtonPress, FALLING);
  attachInterrupt(digitalPinToInterrupt(resetButton), handleResetButtonPress, FALLING);
  Serial.println("Interrupts have been initialized...");
}

void IRAM_ATTR handleButtonPress() {
  isUnlockButtonPressed = true;
  buttonPressCount++;
}

void IRAM_ATTR handleResetButtonPress() {
  isResetButtonPressed = true;  // Set flag
}

void manageButtonPresses() {
  // Local Timer Variables
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = millis();

  // Handle reset button for setup mode
  if (isResetButtonPressed && (currentTime - lastInterruptTime > debounceDelay)) {
    setupPasswordMode = true;
    passwordModeActive = false;
    isProcessAccelActive = true;
    Serial.println("Setup mode activated.");

    isResetButtonPressed = false; 
    lastInterruptTime = currentTime;
  }

  // Handle main button for verification mode
  if (isUnlockButtonPressed && (currentTime - lastInterruptTime > debounceDelay)) {
    passwordModeActive = true;
    setupPasswordMode = false;
    isProcessAccelActive = true;
    Serial.println("Verification mode activated.");

    isUnlockButtonPressed = false;
    lastInterruptTime = currentTime;
  }

  // Error Handling
  if (!passwordModeActive && !setupPasswordMode && !isProcessAccelActive) {
    Serial.println("Ready for next input...");
  }
}

void setupMotionSensor() {
  attachInterrupt(digitalPinToInterrupt(motionSensor), handleMotionInterrupt, RISING);
}

void handleMotionInterrupt() {
  motionDetected = true;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripherals Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setupPeripherals(const int motionSensor, const int pushButton, const int sinusoidLock) {
  // Initialize Motion Sensor
  pinMode(motionSensor, INPUT);

  // Initialize Push Buttons
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(resetButton, INPUT_PULLUP);

  // Initialize Lock Mechanism
  pinMode(sinusoidLock, OUTPUT);
  digitalWrite(sinusoidLock, LOW);

  Serial.println("Perrpherals have been initialized...");
}

void openLockMechanism() {
  static bool isLocked = true;

  if (isLocked) {
    // Unlock Sinusoid
    digitalWrite(sinusoidLock, HIGH);
    Serial.println("Unlock triggered: Door is unlocked.");
    isLocked = false;
    delay(lockDelay);

    // Lock Sinusoid
    digitalWrite(sinusoidLock, LOW);
    Serial.println("Lock triggered: Door is locked.");
    isLocked = true;

    // Reset Flags
    passwordModeActive = false;
    setupPasswordMode = false;
    isProcessAccelActive = false;
    isUnlockButtonPressed = false;
    isResetButtonPressed = false;
    Serial.println("All modes reset after locking.");
  }   
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accelerometer Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setupAccelerometer(float* initialAccelXAxis, float* initialAccelYAxis, float* initialAccelZAxis) {
  // Grab Initial Position
  *initialAccelXAxis = filterAccelXAxisData(*initialAccelXAxis);
  *initialAccelYAxis = filterAccelYAxisData(*initialAccelYAxis);
  *initialAccelZAxis = filterAccelZAxisData(*initialAccelZAxis);
  initializeOrientation(*initialAccelXAxis, *initialAccelYAxis, *initialAccelZAxis);
}

void initializeOrientation(float initialAccelXAxis, float initialAccelYAxis, float initialAccelZAxis) {
  // Local Threshold Variables  
  float movementOrienThreshold = MOVEMENT_COEFFICIENT * GRAVITY;
  float gravityThreshold = GRAVITY_COEFFICIENT * GRAVITY;

  // Determine the Orientation of the Phone
  if (fabs(initialAccelZAxis) > movementOrienThreshold && fabs(initialAccelZAxis) < gravityThreshold){
    if (initialAccelZAxis < BASELINE ? Serial.println("Flat, screen up") : Serial.println("Flat, screen down"));
  } 
  else if (fabs(initialAccelXAxis) > movementOrienThreshold && fabs(initialAccelXAxis) < gravityThreshold){
    if (initialAccelXAxis < BASELINE ? Serial.println("Landscape, left side down") : Serial.println("Landscape, right side down"));
  } 
  else if (fabs(initialAccelYAxis) > movementOrienThreshold and fabs(initialAccelYAxis) < gravityThreshold){
    if (initialAccelYAxis < BASELINE ? Serial.println("Portrait, bottom down") : Serial.println("Portrait, top down"));
  } 
  else {
    Serial.println("Uncertain or in motion");
  }
}

void handleAccelerometerProcessing(){
  // Local Timer Variables
  static unsigned long lastProcessTime = 0;
  const unsigned long processInterval = 500;
  unsigned long currentTime = millis();

  // Process Accelerometer Data
  if (gestureBufferIndex < gestureBufferSize){
    // Process accelerometer data at regular intervals
    if ((currentTime - lastProcessTime) >= processInterval){
      lastProcessTime = currentTime;
      processAccelData(&accelerometerXAxis, &accelerometerYAxis, &accelerometerZAxis);
    }
  } 
  else{
    Serial.println("Password input complete.");
    setupPassword();
    isProcessAccelActive = false;
  }
}

void processAccelData(float* accelerometerXAxis, float* accelerometerYAxis, float* accelerometerZAxis) {
  // Local Timer Variables
  static unsigned long lastUpdate = 0;
  const unsigned long updateInterval = 500;
  unsigned long currentTime = millis();

  // Run Gesture Detection At Predetermined Intervals
  if((currentTime - lastUpdate) >= updateInterval){
    lastUpdate = currentTime;
    *accelerometerXAxis = filterAccelXAxisData(*accelerometerXAxis);
    *accelerometerYAxis = filterAccelYAxisData(*accelerometerYAxis);
    *accelerometerZAxis = filterAccelZAxisData(*accelerometerZAxis);

    detectAccelGesture(*accelerometerXAxis, *accelerometerYAxis, *accelerometerZAxis);
  }
}

float filterAccelXAxisData(float accelXAxis) {
  // Local Timer Variables
  int accelXAxisCounter = 0;
  unsigned long filterXStartTime = millis();

  // Time Alloted to Collect Sensor Data
  while((millis() - filterXStartTime) < filterDelay){
    accelXAxis += Sensor.getAccelerometerXaxis();
    accelXAxisCounter += INCREMENT;
  }

  // No Data Collected
  if (accelXAxisCounter == 0) {
    return 0;
  }

  // Return Filtered Data
  return accelXAxis / (float)accelXAxisCounter;
}

float filterAccelYAxisData(float accelYAxis) {
  // Local Timer Variables
  int accelYAxisCounter = 0;
  unsigned long filterYStartTime = millis();

  // Time Alloted to Collect Sensor Data
  while ((millis() - filterYStartTime) < filterDelay) {
    accelYAxis += Sensor.getAccelerometerYaxis();
    accelYAxisCounter += INCREMENT;
  }

  // No Data Collected
  if (accelYAxisCounter == 0) {
    return 0;
  }

  // Return Filtered Data
  return accelYAxis / (float)accelYAxisCounter;
}

float filterAccelZAxisData(float accelZAxis) {
  // Local Timer Variables
  int accelZAxisCounter = 0;
  unsigned long filterZStartTime = millis();

  // Time Alloted to Collect Sensor Data
  while((millis() - filterZStartTime) < filterDelay){
    accelZAxis += Sensor.getAccelerometerZaxis();
    accelZAxisCounter += INCREMENT;
  }

  // No Data Collected
  if (accelZAxisCounter == 0) {
    return 0;
  }

  // Return Filtered Data
  return accelZAxis / (float)accelZAxisCounter;
}

void printAccelerometerData(float accelXAxis, float accelYAxis, float accelZAxis) {
  Serial.println("---------------------------");
  Serial.println("Accelerometer Data:");
  Serial.printf("X-Axis: %f, Y-Axis: %f, Z-Axis: %f\n", accelXAxis, accelYAxis, accelZAxis);
  Serial.println("---------------------------");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Gesture Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void detectAccelGesture(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis) {
  // Local Timer Variables
  static unsigned long lastGestureTime = 0;
  const unsigned long gestureDebounce = 600;
  unsigned long currentTime = millis();

  // Ignore gestures within debounce time
  if (currentTime - lastGestureTime < gestureDebounce){
    return;
  } 

  // Calculate deltas
  deltaAccelXAxis = filteredAccelXAxis - initialAccelXAxis;
  deltaAccelYAxis = filteredAccelYAxis - initialAccelYAxis;
  deltaAccelZAxis = filteredAccelZAxis - initialAccelZAxis;

  // Update initial positions
  initialAccelXAxis = filteredAccelXAxis;
  initialAccelYAxis = filteredAccelYAxis;
  initialAccelZAxis = filteredAccelZAxis;

  // Boolean variables
  // Threshold Check
  bool thresholdCheckXAxis = fabs(deltaAccelXAxis) > MOVEMENT_THRESHOLD;
  bool thresholdCheckYAxis = fabs(deltaAccelYAxis) > MOVEMENT_THRESHOLD;

  // Compare Axis 
  bool compareXYAxis = fabs(deltaAccelXAxis) > fabs(deltaAccelYAxis);
  bool compareXZAxis = fabs(deltaAccelXAxis) > fabs(deltaAccelZAxis);
  bool compareYXAxis = fabs(deltaAccelYAxis) > fabs(deltaAccelXAxis);
  bool compareYZAxis = fabs(deltaAccelYAxis) > fabs(deltaAccelZAxis);

  // Gesture Detection Checks
  bool gestureXAxisCheck = thresholdCheckXAxis && compareXYAxis && compareXZAxis;
  bool gestureYAxisCheck = thresholdCheckYAxis && compareYXAxis && compareYZAxis;
  bool gestureZAxisCheck = fabs(deltaAccelZAxis) > MOVEMENT_THRESHOLD;

  // Detect gestures
  lastDetectedGesture = noSignificantMovement;
  if (gestureXAxisCheck){
    lastDetectedGesture = (deltaAccelXAxis > BASELINE ? leftToRight : rightToLeft);
  } 
  else if (gestureYAxisCheck){
    lastDetectedGesture = (deltaAccelYAxis > BASELINE ? downToUp : upToDown);
  } 
  else if (gestureZAxisCheck){
    lastDetectedGesture = (deltaAccelZAxis > BASELINE ? centerToBack : centerToForward);
  }

  // Validate and add gesture
  if (lastDetectedGesture != noSignificantMovement) {
    lastGestureTime = currentTime;
    addGestureToBuffer(lastDetectedGesture);
    Serial.printf("Detected Gesture: %d\n", lastDetectedGesture);
  }
}

void clearGestureBuffer() {
  gestureBufferIndex = 0;
  for (int i = 0; i < gestureBufferSize; i++) {
    gestureBuffer[i] = noSignificantMovement;
  }
  
  Serial.println("Gesture buffer cleared.");
}

void addGestureToBuffer(phoneGesture detectedGesture) {
  if (gestureBufferIndex < gestureBufferSize){
    gestureBuffer[gestureBufferIndex++] = detectedGesture;
    Serial.printf("Added gesture: %d at index %d\n", detectedGesture, gestureBufferIndex - 1);
  } 
  else {
    Serial.println("Gesture buffer is full. Resetting...");
    clearGestureBuffer();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Password Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void createSecurityConnection() {
  // Initialize Preferences with read/write mode
  security.begin("passwordBank", false);

  // Retrieve the first password from Preferences
  firstPinCode = security.getString("firstPinCode", "");

  // Debugging: Print out the retrieved PIN codes
  Serial.println("Password system initialized...");
  Serial.print("Retrieved user PIN: ");
  Serial.println(firstPinCode);
  Serial.print("Master PIN: ");
  Serial.println(factoryPin);

  // Display the status of the user PIN
  if (!firstPinCode.isEmpty()) {
    Serial.println("User PIN is stored. Ready for use.");
  } 
  else {
    Serial.println("No user PIN stored. Ready to set a new PIN.");
  }

  // Close Preferences
  security.end();
}

void verifyPassword() {
  // Local Timer and Bool Variables  
  static unsigned long lastGestureTime = 0;
  const unsigned long gestureTimeout = 10000;  // 10 Seconds
  static bool verificationStarted = false;

  // Error handling
  if (!passwordModeActive) {
    verificationStarted = false;
    return;
  }

  // Print the message only once at the start of verification
  if (!verificationStarted) {
    Serial.println("Begin entering gestures for verification...");
    lastGestureTime = millis();
    verificationStarted = true;
  }

  // Check for timeout
  if (millis() - lastGestureTime > gestureTimeout) {
    Serial.println("Gesture entry timeout. Exiting verification mode...");
    clearGestureBuffer();
    passwordModeActive = false;
    verificationStarted = false;
    return;
  }

  // Wait until the buffer is filled with the required number of gestures
  if (gestureBufferIndex < passwordDigits) {
    return;
  }

  // Process and validate the entered PIN
  currentPinCode = ""; 
  for (int i = 0; i < passwordDigits; i++) {
    if (gestureBuffer[i] != noSignificantMovement) {
      currentPinCode += String(static_cast<int>(gestureBuffer[i]));
    }
  }

  // Display the entered PIN
  Serial.print("Entered PIN Code: ");
  Serial.println(currentPinCode);

  // Compare the entered PIN with stored PINs
  if (currentPinCode == firstPinCode) {
    Serial.println("Correct user PIN entered. Unlocking mechanism...");
    openLockMechanism();
  } 
  else if (currentPinCode == factoryPin) {
    Serial.println("Master PIN entered. Unlocking mechanism...");
    openLockMechanism();
  } 
  else {
    Serial.println("Invalid password. Please try again.");
  }

  // Clear the buffer and exit verification mode after processing
  clearGestureBuffer();
  passwordModeActive = false;
  verificationStarted = false;
  isProcessAccelActive = false;
  Serial.println("Exiting Password verification mode...");
}

void setupPassword() {
  static int lastBufferIndex = -1;  // Track the last printed buffer index

  if (gestureBufferIndex < passwordDigits) {
    if (gestureBufferIndex != lastBufferIndex) {
      Serial.printf("Waiting for all gestures... %d/%d entered.\n", gestureBufferIndex, passwordDigits);
      lastBufferIndex = gestureBufferIndex;
    }
    return;
  }

  // Reset the static variable for the next use
  lastBufferIndex = -1;

  // Process the completed PIN
  currentPinCode = "";
  for (int i = 0; i < passwordDigits; i++) {
    if (gestureBuffer[i] != noSignificantMovement) {
      currentPinCode += String(static_cast<int>(gestureBuffer[i]));
    }
  }

  Serial.print("Completed PIN Code: ");
  Serial.println(currentPinCode);

  // Update the PIN regardless of whether it is already set
  updatePinCode(currentPinCode, "firstPinCode");
  Serial.println("User PIN has been updated.");

  // Clear the gesture buffer and exit setup mode
  clearGestureBuffer();
  setupPasswordMode = false;
  isProcessAccelActive = false;
  Serial.println("Exiting Setup Mode...");
}

void updatePinCode(String& pinCode, const String& key) {
  Serial.print("Updating PIN code for key: ");
  Serial.println(key);

  // Open Preferences in write mode
  security.begin("passwordBank", false);

  // Save the new PIN code
  security.putString(key.c_str(), pinCode);
  Serial.print(key);
  Serial.print(" updated with new PIN: ");
  Serial.println(pinCode);

  security.end();  // Close Preferences
  pinCode = "";  // Clear the PIN to avoid reuse
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sleep Mode Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void sleepMode() {
  // Local Timer and Bool Variables
  static unsigned long lastActivityTime = millis();  // Tracks the last activity time
  const unsigned long motionResetDelay = 5000;       // Time to wait before considering motion inactive
  static bool isMotionDetectedRecently = false;      // Tracks recent motion
  unsigned long currentTime = millis();

  // Check if motion is detected
  if (motionDetected) {
    Serial.println("Motion detected. Resetting activity timeout.");
    lastActivityTime = currentTime;
    isMotionDetectedRecently = true;
    motionDetected = false;
  }

  // If no motion and timeout has passed
  if (!isMotionDetectedRecently && (currentTime - lastActivityTime) > motionTimeout) {
    Serial.println("No activity detected. Entering deep sleep mode...");
    esp_deep_sleep_start();
  }

  // Reset recent motion flag if enough time has passed since the last motion
  if (isMotionDetectedRecently && (currentTime - lastActivityTime) > motionResetDelay) {
    isMotionDetectedRecently = false;
  }
}

void initializeSecuritySystem() {
  // Check waking up cause
  esp_sleep_wakeup_cause_t wakeup_cause;
  wakeup_cause = esp_sleep_get_wakeup_cause();
  Serial.println("Deep Sleep initialized...");

  switch (wakeup_cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Woken up by motion!");
      break;
    default:
      Serial.println("System started normally or reset.");
      break;
  }

  // Enable wakeup by the motion sensor
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(motionSensor), LOGICAL_HIGH);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
