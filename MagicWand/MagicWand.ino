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
  - Turn functions into libraries to conserve memory (Do when the system is working completely)
  - Finish setting up peripherals
  - Associate directions with number values 0-6
  - Create entries for two 6-pin passwords
  - Create functions for locking mechanism
  - Sleep mode
  - Auto lock (15 seconds)

  Possible improvements:
  - Voice status
  - LCD display
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Date Created: 10.23.2024
// Last Revision: 11.11.2024
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preprocessor Directives
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ESP32Servo.h> // Version: 3.0.5
#include <DabbleESP32.h> // Version: 1.5.1
#include <EEPROM.h>
#include <math.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define Directives
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Baud Rate Directive
#define BAUD_RATE 115200

// Dabble App Directives
#define CUSTOM_SETTINGS
#define INCLUDE_SENSOR_MODULE

// Database Directives
#define EEPROM_SIZE 64
#define PASSWORD1_ADDR 0
#define PASSWORD2_ADDR 32

// Constant Directives
#define GRAVITY 9.81
#define PI 3.14

// Coefficient Directives
#define MOVEMENT_COEFFICIENT 0.8
#define GRAVITY_COEFFICIENT 1.2

// Conversion Directives
#define ANGLE 180.0

// MISC. Directives
#define SQUARED 2
#define INCREMENT 1
#define MOVEMENT_THRESHOLD 1.0
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ENUM
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum Gesture {
    downToUp,
    upToDown,
    leftToRight,
    rightToLeft,
    centerToBack,
    centerToForward,
    noSignificantMovement = -1
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System Start Routine
void confirmPortConnection();
void confirmDabbleConnection();
void initializeOrientation(float initialAccelXAxis, float initialAccelYAxis, float initialAccelZAxis);
void initializeDistanceSensor(int echoPin, int triggerPin); // Assuming use of HC-SR04
void initializePushButton(int buttonPin);
void initializeLCD(); // Added just in case
void initializeLockMechanism(); //Added just in case

// Collecting Gyroscope Data
float magnitudeGyroscopeData(float filteredGyroXAxis, float filteredGyroYAxis, float filteredGyroZAxis);
float filterGyroXAxisData(float gyroscopeXAxis);
float filterGyroYAxisData(float gyroscopeYAxis);
float filterGyroZAxisData(float gyroscopeZAxis);

// Collecting Accelerometer Data
float magnitudeAccelerometerData(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis);
float filterAccelXAxisData(float accelXAxis);
float filterAccelYAxisData(float accelYAxis);
float filterAccelZAxisData(float accelZAxis);

// Calculate Tilt-Angles
float getYawAngle(float accelXAxis, float accelYAxis, float accelZAxis);
float getPitchAngle(float accelXAxis, float accelYAxis, float accelZAxis);
float getRollAngle(float accelXAxis, float accelYAxis, float accelZAxis);

// Testing Functions
void printGyroscopeData(float gyroscopeXAxis, float gyroscopeYAxis, float gyroscopeZAxis, float gyroMagnitude);
void printAccelerometerData(float accelXAxis, float accelYAxis, float accelZAxis, float accelMagnitude);
void printDirectionAngles(float Alpha, float Beta, float Gamma);
void printTiltAngles(float Pitch, float Roll, float Yaw);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pin Locations
//const int pushBotton = 36;
//const int echoPin;
//const int triggerPin;

// Gyroscope Variables
// Filtered Sensor Data
float gyroscopeXAxis = 0;
float gyroscopeYAxis = 0;
float gyroscopeZAxis = 0;
float gyroRoationMagnitude = 0;

// Accelerometer Variables (negative values follow the direction of gravity, positive is against it)
// Initial Sensor Data
float initialAccelXAxis = 0;
float initialAccelYAxis = 0;
float initialAccelZAxis = 0;

// Filtered Sensor Data
float accelerometerXAxis = 0;
float accelerometerYAxis = 0;
float accelerometerZAxis = 0;
float accelDirectionMagnitude = 0;

// Change in Accelerometer values
float deltaAccelXAxis = 0;
float deltaAccelYAxis = 0;
float deltaAccelZAxis = 0;

// Calculated Tilt Angles
float pitchAngle = 0; // Rotation around the X-axis
float rollAngle = 0; // Rotation around the Y-axis
float yawAngle = 0; // Rotation around the Z-axis

// Calibration Filter Variables
float calibratedPitchAngle = 0;
float calibratedRollAngle = 0;
float filterCoefficient = 0.98;

// Timer Variables
float deltaTime = 0;
unsigned long startTime = 0;
const unsigned long waitTime = 20;
const unsigned long waitASecond = 1000;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Code Drivers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  confirmPortConnection();
  Dabble.begin("MagicWand");
  
  initialAccelXAxis = filterAccelXAxisData(initialAccelXAxis);
  initialAccelYAxis = filterAccelYAxisData(initialAccelYAxis);
  initialAccelZAxis = filterAccelZAxisData(initialAccelZAxis);
  initializeOrientation(initialAccelXAxis, initialAccelYAxis, initialAccelZAxis);
}

void loop() {
  // Needed for Dabble
  confirmDabbleConnection();

  // Filter Gyro Data
  gyroscopeXAxis = filterGyroXAxisData(gyroscopeXAxis);
  gyroscopeYAxis = filterGyroYAxisData(gyroscopeYAxis);
  gyroscopeZAxis = filterGyroZAxisData(gyroscopeZAxis);

  // Filter Accel Data
  accelerometerXAxis = filterAccelXAxisData(accelerometerXAxis);
  accelerometerYAxis = filterAccelYAxisData(accelerometerYAxis);
  accelerometerZAxis = filterAccelZAxisData(accelerometerZAxis);

  // Get Magnitudes
  gyroRoationMagnitude = magnitudeGyroscopeData(gyroscopeXAxis, gyroscopeYAxis, gyroscopeZAxis);
  accelDirectionMagnitude = magnitudeAccelerometerData(accelerometerXAxis, accelerometerYAxis, accelerometerZAxis);

  // Get Tilt-Angles
  yawAngle = getYawAngle(accelerometerXAxis, accelerometerYAxis, accelerometerZAxis);
  pitchAngle = getPitchAngle(accelerometerXAxis, accelerometerYAxis, accelerometerZAxis);
  rollAngle = getRollAngle(accelerometerXAxis, accelerometerYAxis, accelerometerZAxis);

  // Update Phone Orientation
  //updateOrientation(gyroscopeXAxis, gyroscopeYAxis, pitchAngle, rollAngle, deltaTime);
  
  // Debugging Outputs
  printGyroscopeData(gyroscopeXAxis, gyroscopeYAxis, gyroscopeZAxis, gyroRoationMagnitude);
  printAccelerometerData(accelerometerXAxis, accelerometerYAxis, accelerometerZAxis, accelDirectionMagnitude);
  printTiltAngles(pitchAngle, rollAngle, yawAngle);

  switch (gesture) {
    case downToUp:
      Serial.println("Gesture Detected: Down to Up");
      break;
    case upToDown:
      Serial.println("Gesture Detected: Up to Down");
      break;
    case leftToRight:
      Serial.println("Gesture Detected: Left to Right");
      break;
    case rightToLeft:
      Serial.println("Gesture Detected: Right to Left");
      break;
    case centerToBack:
      Serial.println("Gesture Detected: Center to Back");
      break;
    case centerToForward:
      Serial.println("Gesture Detected: Center to Forward");
      break;
    case noSignificantMovement:
      // Optionally handle no movement
      break;
  }
  // Preparing for next iteration
  //startTime = currentTime; 
  delay(6*waitTime);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Function Definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System Start Routine
void confirmPortConnection(){
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  delay(waitTime);
  Serial.println("Serial Port Connection Successful!");
}

void confirmDabbleConnection(){
  Serial.println("Waiting to connect to Dabble...");
  Dabble.waitForAppConnection();
  Serial.println("\nDabble is Connected.");
  delay(waitASecond);
}

void initializePushButton(int buttonPin){ 
  //pinMode(buttonPin, INPUT);  
}

void initializeDistanceSensor(int echoPin, int triggerPin){ // Don't have device yet
 // pinMode(echoPin, INPUT);
 // pinMode(trigPin, OUTPUT);
}

void initializeOrientation(float initialAccelXAxis, float initialAccelYAxis, float initialAccelZAxis) {
    float movementThreshold = MOVEMENT_COEFFICIENT * GRAVITY;
    float gravityThreshold = GRAVITY_COEFFICIENT * GRAVITY;
    int baseline = 0;

    if (abs(initialAccelZAxis) > movementThreshold && abs(initialAccelZAxis) < gravityThreshold) {
      if (initialAccelZAxis > baseline ? Serial.println("Flat, screen up") : Serial.println("Flat, screen down"));
    }
    else if (abs(initialAccelXAxis) > movementThreshold && abs(initialAccelXAxis) < gravityThreshold) {
      if (initialAccelXAxis > baseline ? Serial.println("Landscape, right side down") : Serial.println("Landscape, left side down"));
    } 
    else if (abs(initialAccelYAxis) > movementThreshold && abs(initialAccelYAxis) < gravityThreshold) {
      if (initialAccelYAxis > baseline ? Serial.println("Portrait, bottom down") : Serial.println("Portrait, top down"));
    }
    else {
      Serial.println("Uncertain or in motion");
    }
}

// Collecting Gyroscope Data
float magnitudeGyroscopeData(float filteredGyroXAxis, float filteredGyroYAxis, float filteredGyroZAxis){
  //Square Filtered Gyro Data
  filteredGyroXAxis = pow(filteredGyroXAxis, SQUARED);
  filteredGyroYAxis = pow(filteredGyroYAxis, SQUARED);
  filteredGyroZAxis = pow(filteredGyroZAxis, SQUARED);

  // Return Magnitude
  return sqrt(filteredGyroXAxis + filteredGyroYAxis + filteredGyroZAxis);
}

float filterGyroXAxisData(float gyroscopeXAxis){
  int gyroXAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while((millis() - startTime) < waitTime){
    gyroscopeXAxis += Sensor.getGyroscopeXaxis();
    gyroXAxisCounter += INCREMENT;
  }
  
  // Return Filtered Data
  return gyroscopeXAxis / (float)gyroXAxisCounter;
}

float filterGyroYAxisData(float gyroscopeYAxis){
  int gyroYAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while((millis() - startTime) < waitTime){
    gyroscopeYAxis += Sensor.getGyroscopeYaxis();
    gyroYAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return gyroscopeYAxis / (float)gyroYAxisCounter;
}

float filterGyroZAxisData(float gyroscopeZAxis){
  int gyroZAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while((millis() - startTime) < waitTime){
    gyroscopeZAxis += Sensor.getGyroscopeZaxis();
    gyroZAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return gyroscopeZAxis / (float)gyroZAxisCounter;
}

// Collecting Accelerometer Data
float magnitudeAccelerometerData(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis){
  // Square Filtered Accelerometer Data
  filteredAccelXAxis = pow(filteredAccelXAxis, SQUARED);
  filteredAccelYAxis = pow(filteredAccelYAxis, SQUARED);
  filteredAccelZAxis = pow(filteredAccelZAxis, SQUARED);

  // Return Magnitude
  return sqrt(filteredAccelXAxis + filteredAccelYAxis + filteredAccelZAxis);
}

float filterAccelXAxisData(float accelXAxis){
  int accelXAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while((millis() - startTime) < waitTime){
    accelXAxis += Sensor.getAccelerometerXaxis();
    accelXAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return accelXAxis / (float)accelXAxisCounter;
}

float filterAccelYAxisData(float accelYAxis){
  int accelYAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while((millis() - startTime) < waitTime){
    accelYAxis += Sensor.getAccelerometerYaxis();
    accelYAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return accelYAxis / (float)accelYAxisCounter;
}

float filterAccelZAxisData(float accelZAxis){
  int accelZAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while((millis() - startTime) < waitTime){
    accelZAxis += Sensor.getAccelerometerZaxis();
    accelZAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return accelZAxis / (float)accelZAxisCounter;
}

// Calculate Tilt-Angles
float getYawAngle(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis) {
  // Return Yaw Angle
  return atan2(filteredAccelYAxis, filteredAccelXAxis) * (ANGLE / PI);
}

float getPitchAngle(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis) {
  // Return Pitch Angle
  return atan2(-filteredAccelXAxis, sqrt(pow(filteredAccelYAxis, SQUARED) + pow(filteredAccelZAxis, SQUARED))) * (ANGLE / PI);
}

float getRollAngle(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis) {
  // Return Roll Angle
  return atan2(filteredAccelYAxis, filteredAccelZAxis) * (ANGLE / PI);
}

// Gesture Detection
int detectAccelGesture(float filteredAccelXAxis, float filteredAccelYAxis, float filteredAccelZAxis, float initialAccelXAxis, float initialAccelYAxis, float initialAccelZAxis) {
  deltaAccelXAxis = filteredAccelXAxis - initialAccelXAxis;
  deltaAccelYAxis = filteredAccelYAxis - initialAccelYAxis;
  deltaAccelZAxis = filteredAccelZAxis - initialAccelZAxis;

  if (deltaAccelYAxis > MOVEMENT_THRESHOLD){
    return downToUP;
  }
  if (deltaAccelYAxis < -MOVEMENT_THRESHOLD){
    return upToDown;
  }
  if (deltaAccelXAxis > MOVEMENT_THRESHOLD){
    return leftToRight;
  }
  if (deltaAccelXAxis < -MOVEMENT_THRESHOLD){
    return rightToLeft;
  }
  if (deltaAccelZAxis > MOVEMENT_THRESHOLD){
    return centerToBack;
  }
  if (deltaAccelZAxis < -MOVEMENT_THRESHOLD){
    return centerToForward;
  }

  return noSignificantMovement;
}

// Testing Functions
void printGyroscopeData(float gyroscopeXAxis, float gyroscopeYAxis, float gyroscopeZAxis, float gyroMagnitude){
  Serial.println("---------------------------");
  Serial.println("Gyroscope Data");
  Serial.println("---------------------------");
  Serial.print("X-Axis:"); Serial.print(gyroscopeXAxis); Serial.print(" Y-Axis:"); Serial.print(gyroscopeYAxis); Serial.print(" Z-Axis:"); Serial.println(gyroscopeZAxis); 
  Serial.print("Gyro Magnitude: "); Serial.println(gyroMagnitude);
  Serial.println("---------------------------");
}

void printAccelerometerData(float accelXAxis, float accelYAxis, float accelZAxis, float accelMagnitude){
  Serial.println("---------------------------");
  Serial.println("Accelerometer Data");
  Serial.println("---------------------------");
  Serial.print("X-Axis:"); Serial.print(accelXAxis); Serial.print(" Y-Axis:"); Serial.print(accelYAxis); Serial.print(" Z-Axis:"); Serial.println(accelZAxis); 
  Serial.print("Accel Magnitude: "); Serial.println(accelMagnitude);
  Serial.println("---------------------------");
}

void printTiltAngles(float Pitch, float Roll, float Yaw){
  Serial.println("---------------------------");
  Serial.println("Tilt Angle Data");
  Serial.println("---------------------------");
  Serial.print("Pitch Angle:"); Serial.print(Pitch); Serial.print(" Roll Angle:"); Serial.print(Roll); Serial.print(" Yaw Angle:"); Serial.println(Yaw); 
  Serial.println("---------------------------");  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
