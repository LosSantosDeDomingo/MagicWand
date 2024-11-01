///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Publisher(s): Jose Caraballo, Adrian Suarez
// School: Florida Atlantic University
// Professor: Dr. Alhalabi
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Micro-Controller: NodeMCU ESP-32S
// Arduino IDE Board: NodeMCU-32S
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
  - Test functions
  - Turn functions into libraries to conserve memory
  - Finish setting up peripherals
  - Associate directions with number values 0-6
  - Create entries for two 6-pin passwords
  - Create functions for locking mechanism

  Possible improvements:
  - Sleep mode
  - Auto lock
  - Voice status
  - LCD display
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Date Created: 10.23.2024
// Last Revision: 11.1.2024
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preprocessor Directives
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ESP32Servo.h>
#include <DabbleESP32.h>
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

// MISC. Directives
#define SQUARED 2
#define INCREMENT 1

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System Start Routine
void confirmPortConnection();
void initializeDistanceSensor(int echoPin, int triggerPin); // Assuming use of HC-SR04
void initializePushButton(int buttonPin);
void initializeLCD(); // Added just in case
void initializeLockMechanism(); //Added just in case

// Collecting Gyroscope Data
float magnitudeGyroscopeData(float gyroXAxis, float gyroYAxis, float gyroZAxis);
float filterGyroXAxisData(float gyroXAxis);
float filterGyroYAxisData(float gyroYAxis);
float filterGyroZAxisData(float gyroZAxis);

// Collecting Accelerometer Data
float magnitudeAccelerometerData(float accelXAxis, float accelYAxis, float accelZAxis);
float filterAccelXAxisData(float accelXAxis);
float filterAccelYAxisData(float accelYAxis);
float filterAccelZAxisData(float accelZAxis);

// Calculate 3D-Vector Direction Angles
float getAlphaAngle(float accelXAxis, float magnitudeAccelData);
float getBetaAngle(float accelYAxis, float magnitudeAccelData);
float getGammaAngle(float accelZAxis, float magnitudeAccelData);

// Calculate Tilt-Angles
float getYawAngle(float accelXAxis, float accelYAxis, float accelZAxis);
float getPitchAngle(float accelXAxis, float accelYAxis, float accelZAxis);
float getRollAngle(float accelXAxis, float accelYAxis, float accelZAxis);

// Testing Functions
void printGyroscopeData();
void printAccelerometerData();
void printFilteredGyroData(float gyroXAxis, float gyroYAxis, float gyroZAxis, float gyroMagnitude);
void printFilteredAccelData(float accelXAxis, float accelYAxis, float accelZAxis, float accelMagnitude);
void printDirectionAngles(float Alpha, float Beta, float Gamma);
void printTiltAngles(float Pitch, float Roll, float Yaw);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pin Locations
const int pushBotton = 36;
//const int echoPin;
//const int triggerPin;

// Gyroscope Variables
// Filtered Sensor Data
float gyroXAxisAngle = 0;
float gyroZAxisAngle = 0;
float gyroYAxisAngle = 0;
float gyroRoationMagnitude = 0;

// Accelerometer Variables (negative values follow the direction of gravity, positive is against it)
// Filtered Sensor data
float accelXAxisDirection = 0;
float accelYAxisDirection = 0;
float accelZAxisDirection = 0;
float accelDirectionMagnitude = 0;

// Calculated 3D Vector Direction Angles
float accelAlphaAngle = 0;
float accelBetaAngle = 0;
float accelGammaAngle = 0;

// Calculated Tilt Angles
float pitchAngle = 0; // Rotation around the X-axis
float rollAngle = 0; // Rotation around the Y-axis
float yawAngle = 0; // Rotation around the Z-axis

// Timer Variables
unsigned long startTime = 0;
unsigned long runTime = millis() - startTime;
unsigned long waitTime = 2000; // 2 seconds

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Code Drivers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(BAUD_RATE);
  confirmPortConnection();
  Dabble.begin("MagicWand");
}

void loop() {
  // Needed for Dabble
  Dabble.processInput();

  // Get Magnitudes
  gyroRoationMagnitude = magnitudeGyroscopeData(&gyroXAxisAngle, &gyroYAxisAngle, &gyroZAxisAngle);
  accelDirectionMagnitude = magnitudeAccelerometerData(&accelXAxisDirection, &accelYAxisDirection, &accelZAxisDirection);

  // Get Direction Angles
  accelAlphaAngle = getAlphaAngle(&accelXAxisDirection, &accelDirectionMagnitude);
  accelBetaAngle = getBetaAngle(&accelYAxisDirection, &accelDirectionMagnitude);
  accelGammaAngle = getGammaAngle(&accelZAxisDirection, &accelDirectionMagnitude);

  // Get Tilt-Angles
  yawAngle = getYawAngle(&accelXAxisDirection, &accelYAxisDirection, &accelZAxisDirection);
  pitchAngle = getPitchAngle(&accelXAxisDirection, &accelYAxisDirection, &accelZAxisDirection);
  rollAngle = getRollAngle(&accelXAxisDirection, &accelYAxisDirection, &accelZAxisDirection);

  // Testing Values
  printFilteredGyroData(&gyroXAxisAngle, &gyroYAxisAngle, &gyroZAxisAngle, &gyroRoationMagnitude);
  printFilteredAccelData(&accelXAxisDirection, &accelYAxisDirection, &accelZAxisDirection, &accelDirectionMagnitude);
  printDirectionAngles(&accelAlphaAngle, &accelBetaAngle, &accelGammaAngle);
  printTiltAngles(&PitchAngle, &RollAngle, &YawAngle);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Function Definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System Start Routine
void confirmPortConnection(){
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
}

void initializePushButton(int buttonPin){ 
  pinMode(buttonPin, INPUT);  
}

void initializeDistanceSensor(int echoPin, int triggerPin){ // Don't have device yet
 // pinMode(echoPin, INPUT);
 // pinMode(trigPin, OUTPUT);
}

// Collecting Gyroscope Data
float magnitudeGyroscopeData(float gyroXAxis, float gyroYAxis, float gyroZAxis){
  //Collect Filtered Gyro Data
  float filteredGyroXAxis = filterGyroXAxisData(gyroXAxis);
  float filteredGyroYAxis = filterGyroYAxisData(gyroYAxis);
  float filteredGyroZAxis = filterGyroZAxisData(gyroZAxis);

  //Square Filtered Gyro Data
  filteredGyroXAxis = pow(filteredGyroXAxis, SQUARED);
  filteredGyroYAxis = pow(filteredGyroYAxis, SQUARED);
  filteredGyroZAxis = pow(filteredGyroZAxis, SQUARED);

  // Return Magnitude
  return sqrt(filteredGyroXAxis + filteredGyroYAxis + filteredGyroZAxis);
}

float filterGyroXAxisData(float gyroXAxis){
  int gyroXAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while(runTime < waitTime){
    gyroXAxis += Sensor.getGyroscopeXaxis();
    gyroXAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return gyroXAxis / gyroXAxisCounter;
}

float filterGyroYAxisData(float gyroYAxis){
  int gyroYAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while(runTime < waitTime){
    gyroYAxis += Sensor.getGyroscopeYaxis();
    gyroYAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return gyroYAxis / gyroYAxisCounter;
}

float filterGyroZAxisData(float gyroZAxis){
  int gyroZAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while(runTime < waitTime){
    gyroZAxis += Sensor.getGyroscopeZaxis();
    gyroZAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return gyroZAxis / gyroZAxisCounter;
}

// Collecting Accelerometer Data
float magnitudeAccelerometerData(float accelXAxis, float accelYAxis, float accelZAxis){
  // Collect Filtered Accelerometer Data
  float filteredAccelXAxis = filterAccelXAxisData(accelXAxis);
  float filteredAccelYAxis = filterAccelYAxisData(accelYAxis);
  float filteredAccelZAxis = filterAccelZAxisData(accelZAxis);

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
  while(runTime < waitTime){
    accelXAxis += Sensor.getAccelerometerXaxis();
    accelXAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return accelXAxis / accelXAxisCounter;
}

float filterAccelYAxisData(float accelYAxis){
  int accelYAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while(runTime < waitTime){
    accelYAxis += Sensor.getAccelerometerYaxis();
    accelYAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return accelYAxis / accelYAxisCounter;
}

float filterAccelZAxisData(float accelZAxis){
  int accelZAxisCounter = 0;

  // Time Alloted to Collect Sensor Data
  startTime = millis();
  while(runTime < waitTime){
    accelZAxis += Sensor.getAccelerometerZaxis();
    accelZAxisCounter += INCREMENT;
  }

  // Return Filtered Data
  return accelZAxis / accelZAxisCounter;
}

// Calculate 3D-Vector Direction Angles
float getAlphaAngle(float filteredAccelXAxis, float magnitudeAccelData){
  return acos(filteredAccelXAxis / magnitudeAccelData);
}

float getBetaAngle(float filteredAccelYAxis, float magnitudeAccelData){
  return acos(filteredAccelYAxis / magnitudeAccelData);
}

float getGammaAngle(float filteredAccelZAxis, float magnitudeAccelData){
  return acos(filteredAccelZAxis / magnitudeAccelData);
}

// Calculate Tilt-Angles
float getYawAngle(float accelXAxis, float accelYAxis, float accelZAxis){
  // Collect Filtered Accelerometer Data
  float filteredAccelXAxis = filterAccelXAxisData(accelXAxis);
  float filteredAccelYAxis = filterAccelYAxisData(accelYAxis);
  float filteredAccelZAxis = filterAccelZAxisData(accelZAxis);

  // Square the Filtered Accelerometer Data
  filteredAccelXAxis = pow(filteredAccelXAxis, SQUARED);
  filteredAccelYAxis = pow(filteredAccelYAxis, SQUARED);

  // Return Yaw Angle
  return atan(sqrt(filteredAccelXAxis + filteredAccelYAxis) / filteredAccelZAxis);
}

float getPitchAngle(float accelXAxis, float accelYAxis, float accelZAxis){
  // Collect Filtered Accelerometer Data
  float filteredAccelXAxis = filterAccelXAxisData(accelXAxis);
  float filteredAccelYAxis = filterAccelYAxisData(accelYAxis);
  float filteredAccelZAxis = filterAccelZAxisData(accelZAxis);

  // Square the Filtered Accelerometer Data
  filteredAccelYAxis = pow(filteredAccelYAxis, SQUARED);
  filteredAccelZAxis = pow(filteredAccelZAxis, SQUARED);

  // Return Pitch Angle
  return atan(filteredAccelXAxis / sqrt(filteredAccelYAxis + filteredAccelZAxis));
}

float getRollAngle(float accelXAxis, float accelYAxis, float accelZAxis){
  // Collect Filtered Accelerometer Data
  float filteredAccelXAxis = filterAccelXAxisData(accelXAxis);
  float filteredAccelYAxis = filterAccelYAxisData(accelYAxis);
  float filteredAccelZAxis = filterAccelZAxisData(accelZAxis);

  // Square the Filtered Accelerometer Data
  filteredAccelXAxis = pow(filteredAccelXAxis, SQUARED);
  filteredAccelZAxis = pow(filteredAccelZAxis, SQUARED);
  
    // Return Roll Angle
  return atan(filteredAccelYAxis / sqrt(filteredAccelXAxis + filteredAccelZAxis));
}

// Testing Functions
void printGyroscopeData(){
  Serial.print("X-axis: ");
  Serial.print(Sensor.getGyroscopeXaxis());
  Serial.print('\t');
  Serial.print("Y-axis: ");
  Serial.print(Sensor.getGyroscopeYaxis());
  Serial.print('\t');
  Serial.print("Z-axis: ");
  Serial.println(Sensor.getGyroscopeZaxis());
  Serial.println();
}

void printAccelerometerData(){
  Serial.print("X_axis: ");
  Serial.print(Sensor.getAccelerometerXaxis(), 4);
  Serial.print('\t');
  Serial.print("Y_axis: ");
  Serial.print(Sensor.getAccelerometerYaxis(), 4);
  Serial.print('\t');
  Serial.print("Z_axis: ");
  Serial.println(Sensor.getAccelerometerZaxis(), 4);
  Serial.println();
}

void printFilteredGyroData(float gyroXAxis, float gyroYAxis, float gyroZAxis, float gyroMagnitude){
  println("---------------------------") 
  println("Gyroscope Data");
  println("---------------------------") 
  print("X-Axis: "); print(gyroXAxis); print("Y-Axis: "); print(gyroYAxis); print("Z-Axis: "); println(gyroZAxis); 
  print("Gyro Magnitude: "); println(gyroMagnitude);
  println("---------------------------") 
}

void printFilteredAccelData(float accelXAxis, float accelYAxis, float accelZAxis, float accelMagnitude){
  println("---------------------------") 
  println("Accelerometer Data");
  println("---------------------------") 
  print("X-Axis: "); print(accelXAxis); print("Y-Axis: "); print(accelYAxis); print("Z-Axis: "); println(accelZAxis); 
  print("Accel Magnitude: "); println(accelMagnitude);
  println("---------------------------") 
}

void printDirectionAngles(float Alpha, float Beta, float Gamma){
  println("---------------------------") 
  println("Direction Angle Data");
  println("---------------------------") 
  print("Alpha: "); print(Alpha); print("Beta: "); print(Beta); print("Gamma: "); println(Gamma); 
  println("---------------------------")     
}

void printTiltAngles(float Pitch, float Roll, float Yaw){
  println("---------------------------") 
  println("Tilt Angle Data");
  println("---------------------------") 
  print("Pitch Angle: "); print(Pitch); print("Roll Angle: "); print(Roll); print("Yaw Angle: "); println(Yaw); 
  println("---------------------------")   
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
