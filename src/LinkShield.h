#ifndef LINKSHIELD_H_                     //Include guard
#define LINKSHIELD_H_

//include necessary libraries
#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include <AutomationShield.h>

//Defining pins used by LinkShield
#define LINK_RPIN 0         //Potentiometer pin
#define LINK_UPIN 9         //Servo pin
#define LINK_ServoPIN 1     //Servo position output pin

//Define slave address
#define MPU_6050 0b01101000   //pin AD0 is logic low
//#define MPU_6050 0b01101001 //pin AD0 is logic high

//Define registers address
#define WHO_AM_I    0x75

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
    void  begin(void);                    //initialization function
    float referenceRead();                //r(t)
    void  actuatorWrite(float);           //y(t)
    float sensorAccelXRead();              //u_a(t)
    float sensorAccelYRead();              //u_a(t)
    float sensorAccelZRead();              //u_a(t)
    float sensorGyroXRead();               //u_g(t)
    float sensorGyroYRead();               //u_g(t)
    float sensorGyroZRead();               //u_g(t)
    float servoRead();                    //x(t)
    void  calibrate();                    //Calibration function



  private:

    int16_t _gyro_x_raw;
    int16_t _gyro_y_raw;
    int16_t _gyro_z_raw;

    int16_t _accel_x_raw;
    int16_t _accel_y_raw;
    int16_t _accel_z_raw;

    int     _referenceRead;
    float   _referenceValue;

    int     _servoRead;
    float   _servoValue;

    int16_t read_gyro_X_raw();
    int16_t read_gyro_Y_raw();
    int16_t read_gyro_Z_raw();

    int16_t read_accel_X_raw();
    int16_t read_accel_Y_raw();
    int16_t read_accel_Z_raw();

    float _accelXBias = 0;
    float _accelYBias = 0;
    float _accelZBias = 0;

    float   _gyroXBias = 0;
    float   _gyroYBias = 0;
    float   _gyroZBias = 0;

    float _accelXsum = 0;
    float _accelYsum = 0;
    float _accelZsum = 0;

    float   _gyroXsum = 0;
    float   _gyroYsum = 0;
    float   _gyroZsum = 0;

    float   gyro_lsb_to_degsec = 16.4;
    float   acc_lsb_to_g = 2048.0;

    void    SensorBias(int);

    float _servoTurn;

    bool was_calibrated = 0;

};

LinkClass LinkShield;
Servo myservo;


void LinkClass::begin()
{

  Wire.begin(100);


  // reset MPU 6050
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0b10000000);
  Wire.endTransmission();
  delay(50);

  // awake MPU 6050
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0b00000000);
  Wire.endTransmission();
  delay(10);

  // set sample rate
  Wire.beginTransmission(MPU_6050);
  Wire.write(SMPRT_DIV);
  Wire.write(0b00000000);
  Wire.endTransmission();
  delay(10);

  // configure MPU 6050
  Wire.beginTransmission(MPU_6050);
  Wire.write(CONFIG);
  Wire.write(0b00000000);
  Wire.endTransmission();
  delay(10);

  // configure gyro sensing
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_CONFIG);
  Wire.write(0b00011000);
  Wire.endTransmission();
  delay(10);

  // configure accel sensing
  Wire.beginTransmission(MPU_6050);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0b00011000);
  Wire.endTransmission();
  delay(10);


  myservo.attach(LINK_UPIN, 1000, 2000);

}

//---------------------------------------------------------------------------------

// execute calibration
void LinkClass::calibrate() {
  AutomationShield.serialPrint("Calibration...");
  LinkShield.actuatorWrite(45.0);                    // Go to middle and wait
  delay(500);
  LinkShield.SensorBias(1000);   // Calculate offset
  was_calibrated = 1;
  AutomationShield.serialPrint(" sucessful.\n");
}

//---------------------------------------------------------------------------------

void LinkClass::SensorBias(int testLength) {
  float _accelZsum = 0;
  for (int i = 0; i < testLength; i++ ) {                 //Make N measurements
    _accelXsum  = _accelXsum + LinkShield.sensorAccelXRead();
    _accelYsum  = _accelYsum + LinkShield.sensorAccelYRead();
    _accelZsum  = _accelZsum + LinkShield.sensorAccelZRead();
    _gyroXsum   = _gyroXsum + LinkShield.sensorGyroXRead();
    _gyroYsum   = _gyroYsum + LinkShield.sensorGyroYRead();
    _gyroZsum   = _gyroZsum + LinkShield.sensorGyroZRead();
  }
  _accelXBias = _accelXsum / testLength;     //Compute average
  _accelYBias = _accelYsum / testLength;
  _accelZBias = _accelZsum / testLength;
  _gyroXBias  = _gyroXsum / testLength;
  _gyroYBias  = _gyroYsum / testLength;
  _gyroZBias  = _gyroZsum / testLength;
}

//---------------------------------------------------------------------------------

//values from potentiometer in degrees
float LinkClass::referenceRead() {
  _referenceRead = analogRead(LINK_RPIN);
  _referenceValue = AutomationShield.mapFloat((float)_referenceRead, 0.00, 1023.00, 0.00, 90.00);
  return _referenceValue;
}

//---------------------------------------------------------------------------------

float LinkClass::servoRead() {
  _servoRead = analogRead(LINK_ServoPIN);
  _servoValue = AutomationShield.mapFloat((float)_servoRead, 258.00, 753.00, 0.00, 90.00);
  
  return _servoValue;

}

//---------------------------------------------------------------------------------

int16_t LinkClass::read_gyro_X_raw()
{
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_6050, 2, true);
  _gyro_x_raw = (Wire.read() << 8 | Wire.read());
  return _gyro_x_raw ;
}
int16_t LinkClass::read_gyro_Y_raw()
{
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_YOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_6050, 2, true);
  _gyro_y_raw = (Wire.read() << 8 | Wire.read());
  return _gyro_y_raw ;
}
int16_t LinkClass::read_gyro_Z_raw()
{
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_6050, 2, true);
  _gyro_z_raw = (Wire.read() << 8 | Wire.read());
  return _gyro_z_raw ;
}

//---------------------------------------------------------------------------------

int16_t LinkClass::read_accel_X_raw()
{
  Wire.beginTransmission(MPU_6050);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_6050, 2, true);
  _accel_z_raw = (Wire.read() << 8 | Wire.read());
  return _accel_x_raw ;
}

int16_t LinkClass::read_accel_Y_raw()
{
  Wire.beginTransmission(MPU_6050);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_6050, 2, true);
  _accel_z_raw = (Wire.read() << 8 | Wire.read());
  return _accel_y_raw ;
}

int16_t LinkClass::read_accel_Z_raw()
{
  Wire.beginTransmission(MPU_6050);
  Wire.write(ACCEL_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_6050, 2, true);
  _accel_z_raw = (Wire.read() << 8 | Wire.read());
  return _accel_z_raw ;
}

//---------------------------------------------------------------------------------

void LinkClass::actuatorWrite(float _angle) {
  int _modAngle = map((int)_angle, 0, 90, 0, 180);
  myservo.write(_modAngle);
}

//---------------------------------------------------------------------------------

float LinkClass::sensorAccelXRead() {
  float _accel_X = LinkShield.read_accel_X_raw() / acc_lsb_to_g;
  if (was_calibrated) {
    _accel_X -= _accelXBias;
  }
  return _accel_X;
}

float LinkClass::sensorAccelYRead() {
  float _accel_Y = LinkShield.read_accel_Y_raw() / acc_lsb_to_g;
  if (was_calibrated) {
    _accel_Y -= _accelYBias;
  }
  return _accel_Y;
}

float LinkClass::sensorAccelZRead() {
  float _accel_Z = LinkShield.read_accel_Z_raw() / acc_lsb_to_g;
  if (was_calibrated) {
    _accel_Z -= _accelZBias;
  }
  return _accel_Z;
}

//---------------------------------------------------------------------------------

float LinkClass::sensorGyroXRead() {
  float _gyro_X = LinkShield.read_gyro_X_raw() / gyro_lsb_to_degsec;
  if (was_calibrated) {
    _gyro_X -= _gyroXBias;
  }
  return _gyro_X;
}

float LinkClass::sensorGyroYRead() {
  float  _gyro_Y = LinkShield.read_gyro_Y_raw() / gyro_lsb_to_degsec;
  if (was_calibrated) {
    _gyro_Y -= _gyroYBias;
  }
  return _gyro_Y;
}

float LinkClass::sensorGyroZRead() {
  float  _gyro_Z = LinkShield.read_gyro_Z_raw() / gyro_lsb_to_degsec;
  if (was_calibrated) {
    _gyro_Z -= _gyroZBias;
  }
  return _gyro_Z;
}




#endif
