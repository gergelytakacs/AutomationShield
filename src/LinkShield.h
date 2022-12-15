/*
  API for the LinkShield didactic hardware.

  The file is a part of the application programmers interface for
  the LinkShield didactic tool for control engineering and
  mechatronics education. The LinkShield implements an a flexible
  rotational link experiment on an Arduino shield.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Martin Vríčan.
  Last update: 20.5.2020.
*/

#ifndef LINKSHIELD_H                     //Include guard
#define LINKSHIELD_H

//include necessary libraries                      
#include <Arduino.h>                
#include <Servo.h>
#include "AutomationShield.h"
#include <Wire.h>  

//Defining pins used by LinkShield
#define LINK_RPIN 0         		//Potentiometer pin
#define LINK_FLEX_PIN 1         	//FlexSensor pin
#define LINK_SERVO_ANGLE_PIN 2      //Servo feedback pin
#define LINK_UPIN 9         		//Servo control pin

//Define slave address
#define MPU_6050 0b01101000   //pin AD0 is logic low
//#define MPU_6050 0b01101001 //pin AD0 is logic high

#define SMPRT_DIV 0x19 // Sample rate divider
#define CONFIG 0x1A // DLPF config
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B // Accelerometer measurement
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41 // Temperature measurement
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43 // Gyroscope measurement
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define PWR_MGMT_1 0x6B // Power management


class LinkClass {                         //Creating class
  public:
    void  begin(void);                  //initialization function 
	void  calibrate();                  //Calibration function
	void  actuatorWrite(float);         //u(t)
	
    float referenceRead();              //r(t)
    float flexRead();                  	//y(t)
	float servoRead();
   
    float sensorGyroRead();

  private:
  
    int   _referenceRead;                 
    float _referenceValue;	
	int   _flexRead;                 
    float _flexValue;
    int   _servoRead;                 
    float _servoValue;	

    float _flexSum = 0;
    float _flexBias;
	int   _zeroBendValue = 0;
   
    void  MPU_6050_Init();
	int16_t read_gyro_X_raw();
	int16_t _gyro_x_raw;
	float   gyro_lsb_to_degsec = 16.4;
	
   
   
    float _accelZ;
    float _angle;
    float _sensorBias;                    // Sensor bias from 0
    float _Z_out;
    float flexBias(int);                // Takes N measurements and computes average

};
LinkClass LinkShield;
Servo myservo;

// declaring PIN and initializing sensor library
void LinkClass::begin() {
  myservo.attach(LINK_UPIN, 1000, 2000);    // Set Servo pin and PWM range
  
  #ifdef ARDUINO_ARCH_AVR
  	Wire.begin();	// Starts the "Wire" library for I2C
	analogReference(EXTERNAL); // Set reference voltage (3v3)
  #elif ARDUINO_ARCH_SAM
	//analogReadResolution(12);
	Wire1.begin(); // Initialize I2C communication
  #elif ARDUINO_ARCH_SAMD
        //analogReadResolution(12);
	Wire.begin(); // Initialize I2C communication
  #endif

  MPU_6050_Init();
  
  
  
}

void LinkClass::calibrate() {
  AutomationShield.serialPrint("Calibration...");
  LinkShield.actuatorWrite(45.0);                    // Go to middle and wait
  delay(500);
  _zeroBendValue = flexBias(1000);
  
  
  
  
/*   _sensorBias = -(LinkShield.sensorBias(1000)/4);   // Calculate offset to LSB/g
  ADXL345_OFSZ();  */                                  
  AutomationShield.serialPrint(" successful.\n");
}

float LinkClass::flexBias(int testLength) {
  for (int i = 0; i < testLength; i++ ) {                 //Make N measurements
    _flexSum  = _flexSum + analogRead(LINK_FLEX_PIN);    
  }
  return _flexSum / testLength;                         //Compute average
}

//values from potentiometer in degrees , for fututre use
float LinkClass::referenceRead() {
  _referenceRead = analogRead(LINK_RPIN) - _zeroBendValue;
  _referenceValue = AutomationShield.mapFloat((float)_referenceRead, 0.00, 1023.00, 0.00, 90.00);
  return _referenceValue;
}

float LinkClass::flexRead() {
  _flexRead = analogRead(LINK_FLEX_PIN);
  _flexValue = AutomationShield.mapFloat((float)_flexRead, 0.00, 1023.00, 0.00, 90.00);
  return _flexValue;
}

float LinkClass::servoRead() {
  _servoRead = analogRead(LINK_SERVO_ANGLE_PIN);
  _servoValue = AutomationShield.mapFloat((float)_servoRead, 0.00, 1023.00, 0.00, 90.00);
  return _servoValue;
}

void LinkClass::actuatorWrite(float _angle) {		//Turn servo to desired angle
  int _modAngle = map((int)_angle, 0, 90, 0, 180);
  myservo.write(_modAngle);
}


void LinkClass::MPU_6050_Init(){
	 Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0b10000000);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0b00000000);
  Wire.endTransmission();
  delay(10);


  Wire.beginTransmission(MPU_6050);
  Wire.write(SMPRT_DIV);
  Wire.write(0b00000000);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MPU_6050);
  Wire.write(CONFIG);
  Wire.write(0b00000000);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_CONFIG);
  Wire.write(0b00011000);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MPU_6050);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0b00011000);
  Wire.endTransmission();
}


int16_t LinkClass::read_gyro_X_raw(){
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_6050, 2, true);
  _gyro_x_raw = (Wire.read() << 8 | Wire.read());
  return _gyro_x_raw ;
}

float LinkClass::sensorGyroRead() {
  float  _gyro_X = read_gyro_X_raw() / gyro_lsb_to_degsec;
  return _gyro_X;
}
#endif
