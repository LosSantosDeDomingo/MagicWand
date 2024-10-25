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
  accelerometer and gyroscope from a cellphone. The motions from the phone will follow a 0-9 numbering system.
  Additionally, the program will allow for two programs to be stored within the ESP-32S.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Immediate improvements for current version:
  - Set timer to receive gyroscope and accelerometer data
  - Gather an average amongst the gyroscope data
  - Need to get the magnituges of the gyroscope data
  - 
  - Associate directions with number values

  Possible improvements:
  - 
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Date Created: 10.23.2024
// Last Revision: 10.25.2024
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preprocessor Directives
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ESP32Servo.h>
#include <DabbleESP32.h>
#include <EEPROM.h>
#include <math.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define directive
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Directive for the baud rate
#define BAUD_RATE 115200

// Directives needed for the dabble app
#define CUSTOM_SETTINGS
#define INCLUDE_SENSOR_MODULE

// Directives needed for Database
#define EEPROM_SIZE 64
#define PASSWORD1_ADDR 0
#define PASSWORD2_ADDR 32

// Directive for constants
#define GRAVITY 9.81

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void confirmPortConnection();
void collectGyroscopeData(float gyroXAxis, float gyroYAxis, float gyroZAxis);
float collectGyroXAxisData(float gyroXAxis);
float collectGyroYAxisData(float gyroYAxis);
float collectGyroZAxisData(float gyroZAxis);
void printGyroscopeData();
void printAccelerometerData();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pin locations
const int pushBotton = 36;

// Gyroscope variables
// Sensor data
float gyroscopeXAxisAngle = 0;
float gyroscopeZAxisAngle = 0;
float gyroscopeYAxisAngle = 0;
float gyroscopeRoationMagnitude = 0;

// Accelerometer variables (negative values follow the direction of gravity, positive is against it)
// Sensor data
float accelerometerXAxisDirection = 0;
float accelerometerYAxisDirection = 0;
float accelerometerZAxisDirection = 0;

// Calculated 3D vector direction angle
float accelerometerAlphaAngle = 0;
float accelerometerBetaAngle = 0;
float accelerometerGammaAngle = 0
float accelerometerDirectionMagnitude = 0;

// Calculated tilt angle
float PitchAngle = 0 // Rotation around the X-axis
float RollAngle = 0 // Rotation around the Y-axis
float YawAngle = 0 // Rotation around the Z-axis

// Time
unsigned long start_time = 0;
unsigned long run_time = millis() - start_time;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Code Drivers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(BAUD_RATE);
  confirmPortConnection();
  Dabble.begin("MagicWand");
}

void loop() {
  Dabble.processInput();
  printGyroscopeData();
  printAccelerometerData();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prototype Function Definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void confirmPortConnection(){
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
}

void collectGyroscopeData(float gyroXAxis, float gyroYAxis, float gyroZAxis){
  collectGyroXAxisData(float gyroXAxis);
  collectGyroYAxisData(float gyroYAxis);
  collectGyroZAxisData(float gyroZAxis);
}

float collectGyroXAxisData(float gyroXAxis){
  gyroXAxis = Sensor.getGyroscopeXaxis();
  float filteredGyroXAxis = 0;

}

float collectGyroYAxisData(float gyroYAxis){
  gyroYAxis = Sensor.getGyroscopeYaxis();
  float filteredGyroYAxis = 0;

}

float collectGyroZAxisData(float gyroZAxis){
  gyroZAxis = Sensor.getGyroscopeZaxis();
  float filteredGyroZAxis = 0;
  
}

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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
