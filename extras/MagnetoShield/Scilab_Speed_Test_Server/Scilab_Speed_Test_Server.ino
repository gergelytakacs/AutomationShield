// Modified by "AutomationShield" to contain provisional I2C functionality

#include "Wire.h"

void i2cwrite(byte address, byte command, uint16_t data) { // 16 bits in the form (0,0,0,0,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0)
  Wire.beginTransmission(address);  //addressing
  Wire.write(command);                 // write dac(DAC and EEPROM is 0x60)
  uint8_t firstbyte = (data >> 4); //(0,0,0,0,0,0,0,0,D11,D10,D9,D8,D7,D6,D5,D4) of which only the 8 LSB's survive
  data = data << 12;        //(D3,D2,D1,D0,0,0,0,0,0,0,0,0,0,0,0,0)
  uint8_t secndbyte = (data >> 8); //(0,0,0,0,0,0,0,0,D3,D2,D1,D0,0,0,0,0) of which only the 8 LSB's survive.
  Wire.write(firstbyte); //first 8 MSB's
  Wire.write(secndbyte); //last 4 LSB's
  Wire.endTransmission();
}
/* This file is meant to be used with the SCILAB arduino
   toolbox, however, it can be used from the IDE environment
   (or any other serial terminal) by typing commands like:

   Conversion ascii -> number
   48->'0' ... 57->'9' 58->':' 59->';' 60->'<' 61->'=' 62->'>' 63->'?' 64->'@'
   65->'A' ... 90->'Z' 91->'[' 92->'\' 93->']' 94->'^' 95->'_' 96->'`'
   97->'a' ... 122->'z'

   Dan0 or Dan1 : attach digital pin n (ascii from 2 to b) to input (0) or output (1)
   Drn : read digital value (0 or 1) on pin n (ascii from 2 to b)
   Dwn0 or Dwn1 : write 1 or 0 on pin n
   An    : reads analog pin n (ascii from 0 to 19)
   Wnm  : write analog value m (ascii from 0 to 255) on pin n (ascii from 0 to 19)
   Sa1 or Sa2 : Attach servo 1 (digital pin 9) or 2 (digital pin 10)
   Sw1n or Sw2n : moves servo 1 or servo 2 to position n (from ascii(0) to ascii(180))
   Sd1 or Sd2 : Detach servo 1 or 2

   Generic DC_Motor
   Cijkl : setup for generic DCmotor number i (1 to 4), PW1 on pin number j, PWM2 or direction on pin number k, mode=l
           l=0 for L293 (2 PWM) and l=1 for L298 (1PWM + 1 bit for direction)
   Mijk  : sets speed for generic DCmotor number i, j=0/1 for direction, k=ascii(0) .. ascii(255)
   Mir   : releases motor i (r=release)

   Generic Interrupt counter
   Iai   : activate counter on INT number i (i=ascii(2 or 3 or 18 or 19 or 20 or 21)
   Iri   : release counter on INT number i
   Ipi   : read counter on INT number i
   Izi   : reset counter on INT number i

   MPU6050 acquisition
   Ga : launch acquisition
   Gs : stop acquisition
   Gr : read 9 data in the order yaw, pitch, roll, ax, ay, az, wx, wy ,wz

   Generic Encoder
   Eajkl: activate encoder on channelA on INT number j (j=ascii(2 or 3 or 18 or 19 or 20 or 21) et channelB on pin k or INT number k (k=ascii(0)..ascii(53))
           and l=1 or 2 or 4 for 1x mode (count every rising of chA) or 2x mode (count every change statement of chA)
           or  4x mode (every change statement of chA et chB)
   Eri   : release encoder on INTi
   Epi   : read position of encoder on INTi
   Ezi   : reset value of encoder on INTi position

   R0    : sets analog reference to DEFAULT
   R1    : sets analog reference to INTERNAL
   R2    : sets analog reference to EXTERNAL
   R3   : send version of the code
*/

#include <Servo.h>

#include "Wire.h"
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

/* define internal for the MEGA as 1.1V (as as for the 328)  */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define INTERNAL INTERNAL1V1
#endif

/* create and initialize servos                              */
Servo servo1;
Servo servo2;

/* Generic motors */
int dcm1_pin1, dcm1_pin2, dcm1_mode;
int dcm2_pin1, dcm2_pin2, dcm2_mode;
int dcm3_pin1, dcm3_pin2, dcm3_mode;
int dcm4_pin1, dcm4_pin2, dcm4_mode;

// Generic encoder
/* Encoders initialisation */
// volatile declare as those variables will change in interrupts
volatile long int encoder_0_position = 0, encoder_1_position = 0, encoder_2_position = 0, encoder_3_position = 0, encoder_4_position = 0, encoder_5_position = 0;
int encoder_0_int2 ;          // Pin used for encoder0 channel B : define from scilab
int encoder_1_int2 ;          // Pin used for encoder1 channel B : define from scilab
int encoder_2_int2 ;          // Pin used for encoder2 channel B : define from scilab
int encoder_3_int2 ;          // Pin used for encoder3 channel B : define from scilab
int encoder_4_int2 ;          // Pin used for encoder4 channel B : define from scilab
int encoder_5_int2 ;          // Pin used for encoder5 channel B : define from scilab
int encoder_num, encoder_int2;
int corresp[7] = {0, 2, 3, 21, 20, 19, 18}; //Correspondence between interrupt number and pin number, v5 added a dummy 0 as workaround for ascii 0 issue for scilab

//Generic counter
volatile long int counter_0 = 0, counter_1 = 0, counter_2 = 0, counter_3 = 0, counter_4 = 0, counter_5 = 0;

int initiat = 1;
//
//
//// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//bool acq_launched = false; //boolean used to launch acquisition
//// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorInt16 w;          // [x, y, z]            world-frame gyroscope sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float pr[2];           // [pitch, roll]   pitch/roll container
//float ypr[3];           // [pitch, roll]   pitch/roll container
//
//int data[9];  // lacet, tangage, roulis, ax, ay, az, wx, wy, wz
//
//bool init_mpu=false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  /* initialize serial                                       */
  Serial.begin(115200);
  Wire.begin();
  //
  //     ////////////////////////////
  //    //only if MPU6050 is used
  //    /////////////////////////////
  //   // join I2C bus (I2Cdev library doesn't do this automatically)
  //    Wire.begin();
  //    while (!Serial);
  ////    mpu.initialize();
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    //while (!Serial.available());                 // wait for data
  //    //while (Serial.available() && Serial.read()); // empty buffer again
  //
  //    devStatus = mpu.dmpInitialize();
  //
  //    if (devStatus == 0) {
  //        mpu.setDMPEnabled(true);
  //        attachInterrupt(0, dmpDataReady, RISING);
  //        mpuIntStatus = mpu.getIntStatus();
  //        dmpReady = true;
  //        packetSize = mpu.dmpGetFIFOPacketSize();
  //
  //        init_mpu=true;
  //    }
  //    else {
  //        init_mpu=false;
  //    }
}

void loop() {

  /* variables declaration and initialization                */

  static int  s   = -1;    /* state                          */
  static int  pin = 13;    /* generic pin number             */
  static int  dcm =  4;    /* generic dc motor number        */

  int  val =  0;           /* generic value read from serial */
  int  agv =  0;           /* generic analog value           */
  int  dgv =  0;           /* generic digital value          */
  static int  enc   = 1;    /* encoder number 1 (or 2 for Arduino mega)     */
  //
  // if (true){
  //  get_data_MPU6050();
  // }
  if (Serial.available() != 0) {
    val = Serial.read();
  };

  //while (Serial.available()==0) {}; // Waiting char
  //val = Serial.read();

  //  if (val==0){// version
  //    Serial.print('v3');
  //    val=-1;
  //  }
  //case A -> Analog
  if (val == 65) { //A -> Analog read
    while (Serial.available() == 0) {}; // Waiting char
    //    val=Serial.read();
    //    if (val==114){ //'r'-> read pin
    //       while (Serial.available()==0) {}; // Waiting char
    val = Serial.read();
    if (val > 47 && val < 67) { //from pin 0, to pin 19
      pin = val - 48; //number of the pin
      agv = analogRead(pin);
      //Serial.println(agv);
      //Serial.print(agv);
      Serial.write((uint8_t*)&agv, 2); /* send binary value via serial  */
    }
    val = -1;
  }
  else if (val == 87) { //W -> Analog write
    while (Serial.available() == 0) {}; // Waiting char
    val = Serial.read();
    if (val > 47 && val < 67) { //from pin 0 to pin 19
      pin = val - 48; //number of the pin
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read();
      analogWrite(pin, val);
    }
    val = -1;
  }
  //}

  //case I -> Provisional I2C
if (val == 73) { //I -> Provisional I2C
    while (Serial.available() == 0) {}; // Waiting for address
    byte address = Serial.read();
    while (Serial.available() == 0) {}; // Waiting for command
    byte command = Serial.read();
    while (Serial.available() == 0) {}; // Waiting for data1
    byte data1 = Serial.read();
    while (Serial.available() == 0) {}; // Waiting for data2
    byte data2 = Serial.read();
    uint16_t data = word(data1, data2);
    i2cwrite(address, command, data);
    val = -1;
}

  //case D -> Digital
  else if (val == 68) { //D -> Digital pins
    while (Serial.available() == 0) {}; // Waiting char
    val = Serial.read();
    if (val == 97) { //'a'-> declare pin
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read();
      if (val > 49 && val < 102) {
        pin = val - 48;
        while (Serial.available() == 0) {}; // Waiting char
        val = Serial.read();
        if (val == 48 || val == 49) {
          if (val == 48) { //'0' -> input
            pinMode(pin, INPUT);
          }
          else if (val == 49) { //'1' -> output
            pinMode(pin, OUTPUT);
          }
        }
      }
    }
    if (val == 114) { //'r'-> read pin
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read();
      if (val > 49 && val < 102) {
        pin = val - 48; //number of the digital pin
        dgv = digitalRead(pin);
        //          Serial.println(dgv);
        Serial.print(dgv);
      }
    }
    if (val == 119) { //'w'-> write pin
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read();
      if (val > 49 && val < 102) {
        pin = val - 48; //number of the digital pin
        while (Serial.available() == 0) {}; // Waiting char
        val = Serial.read();
        if (val == 48 || val == 49) { // 0 or 1
          dgv = val - 48;
          digitalWrite(pin, dgv);
          //            Serial.println(dgv);
        }
      }
    }
    val = -1;

  }

  //case S -> servomotor
  else if (val == 83) {
    while (Serial.available() == 0) {}; // Waiting char
    val = Serial.read();
    if (val == 97) { //'a'-> declare servo
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read();
      if (val == 49 || val == 50) { //servo 1 or 2
        pin = val - 48; //number of the servo
        if (pin == 1) {
          servo1.attach(9);
          servo1.write(0);
          //            agv=servo1.read();
          //            Serial.println(agv);
        }
        if (pin == 2) {
          servo2.attach(10);
          servo2.write(0);
          //            agv=servo2.read();
          //            Serial.println(agv);
        }

      }
    }
    if (val == 100) { //'d'-> detach servo
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read();
      if (val == 49 || val == 50) { //servo 1 or 2
        pin = val - 48; //number of the servo
        if (pin == 1) {
          servo1.detach();
        }
        if (pin == 2) {
          servo2.detach();
        }
      }
    }
    if (val == 119) { //'w'-> write pin
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read();
      if (val == 49 || val == 50) { //servo 1 or 2
        pin = val - 48; //number of the servo
        while (Serial.available() == 0) {}; // Waiting char
        val = Serial.read();
        if (val >= 0 && val <= 180) {
          if (pin == 1) {
            servo1.write(val);
            //                agv=servo1.read();
            //                Serial.println(agv);
          }
          if (pin == 2) {
            servo2.write(val);
            //                agv=servo2.read();
            //                Serial.println(agv);
          }
        }
      }
    }
    val = -1;

  }
  //case G -> deal with MPU6050
  //  else if (val==71){
  //    while (Serial.available()==0) {}; // Waiting char
  //      val=Serial.read();
  //      if (val==116){ //t = test if initialisation is ok
  //        if (init_mpu==true) Serial.print("OK");
  //        else Serial.print("KO");
  //      }
  //      if (val==97) { //a = activation (acquisition is launched)
  //        acq_launched= true;
  //        val=-1;
  //      }
  //      if (val==114) { //r = read
  //        //get_data_MPU6050();
  //        if(false){
  //        for(int d=0;d<9;d++){
  //          Serial.print(data[d]);
  //          Serial.print(" ; ");
  //        }
  //        Serial.println(" fin");
  //        }
  //        else{
  //              Serial.write((uint8_t*) &data,2*9);
  //              /*Serial.write((uint8_t*) &ypr,3*4);
  //              Serial.write((uint8_t*)&aa.x,2);
  //              Serial.write((uint8_t*)&aa.y,2);
  //              Serial.write((uint8_t*)&aa.z,2);
  //              Serial.write((uint8_t*)&w.x,2);
  //              Serial.write((uint8_t*)&w.y,2);
  //              Serial.write((uint8_t*)&w.z,2); */
  //        }
  //
  //
  //        val=-1;
  //
  //      }
  //      if (val==115) { //s = stop (acquisition is stopped)
  //        acq_launched= false;
  //        val=-1;
  //      }
  //
  //  }
  //case I -> Interrupt
  else if (val == 73) {
    /* ASKING ACTIVATION OF AN COUNTER     */
    while (Serial.available() == 0) {}; // Waiting char
    val = Serial.read();
    if (val == 97) { //a = activation
      while (Serial.available() == 0) {};               // Waiting char
      val = Serial.read();                              // Read int_number (must be 0 or 1 on UNO / 1 to 5 on MEGA) : int_number set to encoder number
      pinMode(corresp[val], INPUT);             // set interrupt pin as input
      if       (val == 0) {
        attachInterrupt(val, counter_0_change, RISING);  //counter INT0
        counter_0 = 0;
      }
      else if  (val == 1) {
        attachInterrupt(val, counter_1_change, RISING);  //counter INT1
        counter_1 = 0;
      }
      else if  (val == 2) {
        attachInterrupt(val, counter_2_change, RISING);  //counter INT2
        counter_2 = 0;
      }
      else if  (val == 3) {
        attachInterrupt(val, counter_3_change, RISING);  //counter INT3
        counter_3 = 0;
      }
      else if  (val == 4) {
        attachInterrupt(val, counter_4_change, RISING);  //counter INT4
        counter_4 = 0;
      }
      else if  (val == 5) {
        attachInterrupt(val, counter_5_change, RISING);  //counter INT5
        counter_5 = 0;
      }
    }
    /* ASKING POSITION OF A COUNTER     */
    if (val == 112) { //p = sending counting value
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read()   ;             //reading next value = counter number
      if      (val == 0) {
        Serial.write((uint8_t*)&counter_0, 4);  // asking counter 0
      }
      else if (val == 1) {
        Serial.write((uint8_t*)&counter_1, 4);  // asking counter 1
      }
      else if (val == 2) {
        Serial.write((uint8_t*)&counter_2, 4);  // asking counter 2
      }
      else if (val == 3) {
        Serial.write((uint8_t*)&counter_3, 4);  // asking counter 3
      }
      else if (val == 4) {
        Serial.write((uint8_t*)&counter_4, 4);  // asking counter 4
      }
      else if (val == 5) {
        Serial.write((uint8_t*)&counter_5, 4);  // asking counter 5
      }
    }
    /* ASKING RELEASE OF AN INTERRUPT   */
    if (val == 114) { //r = release counter
      while (Serial.available() == 0) {};      // Waiting char
      val = Serial.read();                     //reading next value = counter number
      detachInterrupt(val);                    // Detach interrupt on channel A of counter num=val
      if (val == 0)       {
        counter_0 = 0; // Reset counter
      }
      else if (val == 1)  {
        counter_1 = 0; // Reset counter
      }
      else if (val == 2)  {
        counter_2 = 0; // Reset counter
      }
      else if (val == 3)  {
        counter_3 = 0; // Reset counter
      }
      else if (val == 4)  {
        counter_4 = 0; // Reset counter
      }
      else if (val == 5)  {
        counter_5 = 0; // Reset counter
      }
    }
    /* ASKING RESET VALUE OF AN COUNTER     */
    if (val == 122) { //z set to zero
      while (Serial.available() == 0) {};      // Waiting char
      val = Serial.read();                     //reading next value = counter number
      if (val == 0)       {
        counter_0 = 0; // Reset counter
      }
      else if (val == 1)  {
        counter_1 = 0; // Reset counter
      }
      else if (val == 2)  {
        counter_2 = 0; // Reset counter
      }
      else if (val == 3)  {
        counter_3 = 0; // Reset counter
      }
      else if (val == 4)  {
        counter_4 = 0; // Reset counter
      }
      else if (val == 5)  {
        counter_5 = 0; // Reset counter
      }
    }
    val = -1;

  }

  //case E -> Encoder
  else if (val == 69) {
    /*Generic encoder functions */
    while (Serial.available() == 0) {};
    val = Serial.read();
    /* ASKING ACTIVATION OF AN ENCODER     */
    if (val == 97) {                                     //activation
      while (Serial.available() == 0) {};               // Waiting char
      encoder_num = Serial.read();                      // Read int_number (must be 0 or 1 on UNO / 1 to 5 on MEGA) : int_number set to encoer number
      pinMode(corresp[encoder_num], INPUT);             // set interrupt pin as input
      while (Serial.available() == 0) {};               // Waiting char
      encoder_int2 = Serial.read();                     // Read int2 (must be a digital PIN with interrupt or not : depends on mode)
      // no declaration for the moment : wait for encoder mode
      while (Serial.available() == 0) {};               // Waiting char
      int mode = Serial.read() - 48;                    // Read mode 1 ou 2 (1 counting only rising of chA, 2 counting rising and falling)
      if (mode == 4) {                                  // mode 4x : 2 cases : chA=pin2 / chB=pin3 or chA=pin3/chB=pin2 [Uno restriction]
        pinMode(corresp[encoder_int2], INPUT);         // set interrupt number as input
      } else {
        pinMode(encoder_int2, INPUT);                  // set pin as input
      }

      if (encoder_num == 0) {                             //encoder INT0
        encoder_0_position = 0;                           // Reset position
        if (mode == 4) {
          encoder_0_int2 = corresp[encoder_int2];         // Save pin of second interruption
          attachInterrupt(encoder_num , encoder_change_m4_A0, CHANGE); // Attach interrupt on channel A change
          attachInterrupt(encoder_int2, encoder_change_m4_B0, CHANGE); // Attach interrupt on channel B change
        } else if (mode == 2) {
          encoder_0_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_0_change_m2, CHANGE); // Attach interrupt on channel A change
        } else if (mode == 1) {
          encoder_0_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_0_change_m1, RISING); // Attach interrupt on channel A rising
        }
      } else if (encoder_num == 1) {                      //encoder INT1
        encoder_1_position = 0;                           // Reset position
        if (mode == 4) {
          encoder_1_int2 = corresp[encoder_int2];         // Save pin of second interruption
          attachInterrupt(encoder_num , encoder_change_m4_A1, CHANGE); // Attach interrupt on channel A change
          attachInterrupt(encoder_int2, encoder_change_m4_B1, CHANGE); // Attach interrupt on channel B change
        } else if (mode == 2) {
          encoder_1_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_1_change_m2, CHANGE); // Attach interrupt on channel A change
        } else if (mode == 1) {
          encoder_1_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_1_change_m1, RISING); // Attach interrupt on channel A rising
        }
      } else if (encoder_num == 2) {                      //encoder INT2
        encoder_2_position = 0;                           // Reset position
        if (mode == 4) {
          encoder_2_int2 = corresp[encoder_int2];         // Save pin of second interruption
          attachInterrupt(encoder_num , encoder_change_m4_A2, CHANGE); // Attach interrupt on channel A change
          attachInterrupt(encoder_int2, encoder_change_m4_B2, CHANGE); // Attach interrupt on channel B change
        } else if (mode == 2) {
          encoder_2_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_2_change_m2, CHANGE); // Attach interrupt on channel A change
        } else if (mode == 1) {
          encoder_2_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_2_change_m1, RISING); // Attach interrupt on channel A rising
        }
      } else if (encoder_num == 3) {                      //encoder INT3
        encoder_3_position = 0;                           // Reset position
        if (mode == 4) {
          encoder_3_int2 = corresp[encoder_int2];         // Save pin of second interruption
          attachInterrupt(encoder_num , encoder_change_m4_A3, CHANGE); // Attach interrupt on channel A change
          attachInterrupt(encoder_int2, encoder_change_m4_B3, CHANGE); // Attach interrupt on channel B change
        } else if (mode == 2) {
          encoder_3_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_3_change_m2, CHANGE); // Attach interrupt on channel A change
        } else if (mode == 1) {
          encoder_3_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_3_change_m1, RISING); // Attach interrupt on channel A rising
        }
      } else if (encoder_num == 4) {                      //encoder INT4
        encoder_4_position = 0;                           // Reset position
        if (mode == 4) {
          encoder_4_int2 = corresp[encoder_int2];         // Save pin of second interruption
          attachInterrupt(encoder_num , encoder_change_m4_A4, CHANGE); // Attach interrupt on channel A change
          attachInterrupt(encoder_int2, encoder_change_m4_B4, CHANGE); // Attach interrupt on channel B change
        } else if (mode == 2) {
          encoder_4_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_4_change_m2, CHANGE); // Attach interrupt on channel A change
        } else if (mode == 1) {
          encoder_4_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_4_change_m1, RISING); // Attach interrupt on channel A rising
        }
      } else if (encoder_num == 5) {                      //encoder INT5
        encoder_5_position = 0;                           // Reset position
        if (mode == 4) {
          encoder_5_int2 = corresp[encoder_int2];         // Save pin of second interruption
          attachInterrupt(encoder_num , encoder_change_m4_A5, CHANGE); // Attach interrupt on channel A change
          attachInterrupt(encoder_int2, encoder_change_m4_B5, CHANGE); // Attach interrupt on channel B change
        } else if (mode == 2) {
          encoder_5_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_5_change_m2, CHANGE); // Attach interrupt on channel A change
        } else if (mode == 1) {
          encoder_5_int2 = encoder_int2;
          attachInterrupt(encoder_num, encoder_5_change_m1, RISING); // Attach interrupt on channel A rising
        }
      }
    }
    /* ASKING POSITION OF AN ENCODER     */
    if (val == 112) { //p = sending encoder position
      while (Serial.available() == 0) {}; // Waiting char
      val = Serial.read()   ;             //reading next value = encoder number
      if      (val == 0) {
        Serial.write((uint8_t*)&encoder_0_position, 4);  // asking encoder 0 position
      }
      else if (val == 1) {
        Serial.write((uint8_t*)&encoder_1_position, 4);  // asking encoder 1 position
      }
      else if (val == 2) {
        Serial.write((uint8_t*)&encoder_2_position, 4);  // asking encoder 2 position
      }
      else if (val == 3) {
        Serial.write((uint8_t*)&encoder_3_position, 4);  // asking encoder 3 position
      }
      else if (val == 4) {
        Serial.write((uint8_t*)&encoder_4_position, 4);  // asking encoder 4 position
      }
      else if (val == 5) {
        Serial.write((uint8_t*)&encoder_5_position, 4);  // asking encoder 5 position
      }
    }
    /* ASKING RELEASE OF AN ENCODER     */
    if (val == 114) { //r = release encoder
      while (Serial.available() == 0) {};               // Waiting char
      val = Serial.read();                              //reading next value = encoder number
      detachInterrupt(val);                             // Detach interrupt on channel A of encoder num=val
      if (val == 0)       {
        encoder_0_position = 0;  // Reset position
        encoder_0_int2 = -1;
      }
      else if (val == 1)  {
        encoder_1_position = 0;  // Reset position
        encoder_1_int2 = -1;
      }
      else if (val == 2)  {
        encoder_2_position = 0;  // Reset position
        encoder_2_int2 = -1;
      }
      else if (val == 3)  {
        encoder_3_position = 0;  // Reset position
        encoder_3_int2 = -1;
      }
      else if (val == 4)  {
        encoder_4_position = 0;  // Reset position
        encoder_4_int2 = -1;
      }
      else if (val == 5)  {
        encoder_5_position = 0;  // Reset position
        encoder_5_int2 = -1;
      }
      while (Serial.available() == 0) {};               // Waiting char
      val = Serial.read();                              // reading next value = encoder number
      detachInterrupt(val);                             // Detach interrupt on channel B of encoder num=val (may be the same if mode=1 or 2)
    }
    /* ASKING RESET POSITION OF AN ENCODER     */
    if (val == 122) { // z = encoder position to zero
      while (Serial.available() == 0) {};               // Waiting char
      val = Serial.read();                              //reading next value = encoder number
      if (val == 0)       {
        encoder_0_position = 0; // Reset position
      }
      else if (val == 1)  {
        encoder_1_position = 0; // Reset position
      }
      else if (val == 2)  {
        encoder_2_position = 0; // Reset position
      }
      else if (val == 3)  {
        encoder_3_position = 0; // Reset position
      }
      else if (val == 4)  {
        encoder_4_position = 0; // Reset position
      }
      else if (val == 5)  {
        encoder_5_position = 0; // Reset position
      }
    }
    val = -1;

  }

  //case C -> DCmotor init
  else if (val == 67) {
    while (Serial.available() == 0) {};
    val = Serial.read();
    /* 2nd char = motor number */
    if (val > 48 && val < 53) {
      dcm = val - 48;
      while (Serial.available() == 0) {};
      val = Serial.read();
      /* the third received value indicates the pin1 number from ascii(2)=50 to ascii(e)=101  */
      if (val > 49 && val < 102) {
        if (dcm == 1) dcm1_pin1 = val - 48; /* calculate motor pin1 */
        if (dcm == 2) dcm2_pin1 = val - 48; /* calculate motor pin1 */
        if (dcm == 3) dcm3_pin1 = val - 48; /* calculate motor pin1 */
        if (dcm == 4) dcm4_pin1 = val - 48; /* calculate motor pin1 */
        pinMode(val - 48, OUTPUT); //set pin as output
        analogWrite(val - 48, 0); /* DUTY CYCLE */
        while (Serial.available() == 0) {};
        val = Serial.read();
        /* the fourth received value indicates the pin2 number from ascii(2)=50 to ascii(e)=101  */
        if (val > 49 && val < 102) {
          if (dcm == 1) dcm1_pin2 = val - 48; /* calculate motor pin2 */
          if (dcm == 2) dcm2_pin2 = val - 48; /* calculate motor pin2 */
          if (dcm == 3) dcm3_pin2 = val - 48; /* calculate motor pin2 */
          if (dcm == 4) dcm4_pin2 = val - 48; /* calculate motor pin2 */
          pinMode(val - 48, OUTPUT); //set pin as output
          while (Serial.available() == 0) {};
          val = Serial.read();
          /* the fifth received value indicates the pin2 number from ascii(2)=50 to ascii(e)=101 */
          if (val > 47 && val < 50) {
            int mode = val - 48;
            if (dcm == 1) dcm1_mode = mode; /* calculate motor mode */
            if (dcm == 2) dcm2_mode = mode; /* calculate motor mode */
            if (dcm == 3) dcm3_mode = mode; /* calculate motor mode */
            if (dcm == 4) dcm4_mode = mode; /* calculate motor mode */
            //initialization of port
            if (mode == 0) { //L293
              if (dcm == 1) analogWrite(dcm1_pin2, 0); /* DUTY CYCLE */
              if (dcm == 2) analogWrite(dcm2_pin2, 0); /* DUTY CYCLE */
              if (dcm == 3) analogWrite(dcm3_pin2, 0); /* DUTY CYCLE */
              if (dcm == 4) analogWrite(dcm4_pin2, 0); /* DUTY CYCLE */
            } else if (mode == 1) { //L297
              if (dcm == 1) digitalWrite(dcm1_pin2, LOW); /* DIRECTION */
              if (dcm == 2) digitalWrite(dcm2_pin2, LOW); /* DIRECTION */
              if (dcm == 3) digitalWrite(dcm3_pin2, LOW); /* DIRECTION */
              if (dcm == 4) digitalWrite(dcm4_pin2, LOW); /* DIRECTION */
            }
            Serial.print("OK"); // tell Scilab that motor s initialization finished
            // Cette commande sert à rien dans la toolbox de base,
            // sauf si on prévoit d'ajouter des actions à l'init des motors
            // par example chercher la position d'origine !
          }
        }
      }
    }
    val = -1;

  }

  //case M -> DC motor
  else if (val == 77) {
    while (Serial.available() == 0) {};
    val = Serial.read();
    /* the second received value indicates the motor number
       from abs('1')=49, motor1, to abs('4')=52, motor4    */
    if (val > 48 && val < 53) {
      dcm = val - 48;            /* calculate motor number */
      while (Serial.available() == 0) {};     // Waiting char
      val = Serial.read();
      /* the third received value indicates the sens direction or release*/
      if (val == 48 || val == 49) {
        int direction = val - 48;
        while (Serial.available() == 0) {};     // Waiting char
        val = Serial.read();                    //reading next value = 0..255
        if (dcm == 1) {
          if (dcm1_mode == 0) { //L293
            if (direction == 1) {
              analogWrite(dcm1_pin1, val);
              analogWrite(dcm1_pin2, 0);
            } else {
              analogWrite(dcm1_pin2, val);
              analogWrite(dcm1_pin1, 0);
            }
          } else {//L298
            if (direction == 0) digitalWrite(dcm1_pin2, LOW);
            if (direction == 1) digitalWrite(dcm1_pin2, HIGH);
            analogWrite(dcm1_pin1, val);
            //Timer3.pwm(dcm1_pin1, val*4);
          }
        }
        if (dcm == 2) {
          if (dcm2_mode == 0) { //L293
            if (direction == 1) {
              analogWrite(dcm2_pin1, val);
              analogWrite(dcm2_pin2, 0);
            } else {
              analogWrite(dcm2_pin2, val);
              analogWrite(dcm2_pin1, 0);
            }
          } else {//L298
            if (direction == 0) digitalWrite(dcm2_pin2, LOW);
            if (direction == 1) digitalWrite(dcm2_pin2, HIGH);
            analogWrite(dcm2_pin1, val);
            //Timer3.pwm(dcm2_pin1, val*4);
          }
        }
        if (dcm == 3) {
          if (dcm3_mode == 0) { //L293
            if (direction == 1) {
              analogWrite(dcm3_pin1, val);
              analogWrite(dcm3_pin2, 0);
            } else {
              analogWrite(dcm3_pin2, val);
              analogWrite(dcm3_pin1, 0);
            }
          } else {//L298
            if (direction == 0) digitalWrite(dcm3_pin2, LOW);
            if (direction == 1) digitalWrite(dcm3_pin2, HIGH);
            analogWrite(dcm3_pin1, val);
          }
        }
        if (dcm == 4) {
          if (dcm4_mode == 0) { //L293
            if (direction == 1) {
              analogWrite(dcm4_pin1, val);
              analogWrite(dcm4_pin2, 0);
            } else {
              analogWrite(dcm4_pin2, val);
              analogWrite(dcm4_pin1, 0);
            }
          } else {//L298
            if (direction == 0) digitalWrite(dcm4_pin2, LOW);
            if (direction == 1) digitalWrite(dcm4_pin2, HIGH);
            analogWrite(dcm4_pin1, val);
          }
        }
      }
      if (val == 114) { //release motor
        if (dcm == 1) {
          analogWrite(dcm1_pin1, 0);
          if (dcm1_mode == 0)  analogWrite(dcm1_pin2, 0);
        }
        if (dcm == 2) {
          analogWrite(dcm2_pin1, 0);
          if (dcm2_mode == 0)  analogWrite(dcm2_pin2, 0);
        }
        if (dcm == 3) {
          analogWrite(dcm3_pin1, 0);
          if (dcm3_mode == 0)  analogWrite(dcm3_pin2, 0);
        }
        if (dcm == 4) {
          analogWrite(dcm4_pin1, 0);
          if (dcm4_mode == 0)  analogWrite(dcm4_pin2, 0);
        }
      }

    }
    val = -1;

  }


  //case R -> Analog reference
  if (val == 82) {
    while (Serial.available() == 0) {};
    val = Serial.read();
    if (val == 48) analogReference(DEFAULT);
    if (val == 49) analogReference(INTERNAL);
    if (val == 50) analogReference(EXTERNAL);
    if (val == 51) Serial.print("v5");
    val = -1;
  }

} /* end loop statement                                      */


/**************************************/
// Generic interrupt encoder functions//
/**************************************/
//Encoder on INT0
void encoder_0_change_m1() { //encoder0 mode 1x
  int chB = digitalRead(encoder_0_int2);
  if (!chB) {
    encoder_0_position++;
  }
  else {
    encoder_0_position--;
  }
}
void encoder_0_change_m2() { //encoder0 mode 2x
  int chB = digitalRead(encoder_0_int2);
  int chA = digitalRead(corresp[0]);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_0_position++;
  }
  else {
    encoder_0_position--;
  }
}
void encoder_change_m4_A0() { //encoder0 mode 4x chA
  int chA = digitalRead(corresp[0]);
  int chB = digitalRead(encoder_0_int2);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_0_position++;
  }
  else {
    encoder_0_position--;
  }
}
void encoder_change_m4_B0() { //encoder0 mode 4x chB
  int chA = digitalRead(corresp[0]);
  int chB = digitalRead(encoder_0_int2);
  if ((!chA & !chB) | (chA & chB)) {
    encoder_0_position++;
  }
  else {
    encoder_0_position--;
  }
}
//Encoder on INT1
void encoder_1_change_m1() { //encoder1 mode 1x
  int chB = digitalRead(encoder_1_int2);
  if (!chB) {
    encoder_1_position++;
  }
  else {
    encoder_1_position--;
  }
}
void encoder_1_change_m2() { //encoder1 mode 2x
  int chB = digitalRead(encoder_1_int2);
  int chA = digitalRead(corresp[1]);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_1_position++;
  }
  else {
    encoder_1_position--;
  }
}
void encoder_change_m4_A1() { //encoder1 mode 4x chA
  int chA = digitalRead(corresp[1]);
  int chB = digitalRead(encoder_1_int2);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_1_position++;
  }
  else {
    encoder_1_position--;
  }
}
void encoder_change_m4_B1() { //encoder1 mode 4x chB
  int chA = digitalRead(corresp[1]);
  int chB = digitalRead(encoder_1_int2);
  if ((!chA & !chB) | (chA & chB)) {
    encoder_1_position++;
  }
  else {
    encoder_1_position--;
  }
}
//Encoder on INT2
void encoder_2_change_m1() { //encoder2 mode 1x
  int chB = digitalRead(encoder_2_int2);
  if (!chB) {
    encoder_2_position++;
  }
  else {
    encoder_2_position--;
  }
}
void encoder_2_change_m2() { //encoder2 mode 2x
  int chB = digitalRead(encoder_2_int2);
  int chA = digitalRead(corresp[2]);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_2_position++;
  }
  else {
    encoder_2_position--;
  }
}
void encoder_change_m4_A2() { //encoder2 mode 4x chA
  int chA = digitalRead(corresp[2]);
  int chB = digitalRead(encoder_2_int2);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_2_position++;
  }
  else {
    encoder_2_position--;
  }
}
void encoder_change_m4_B2() { //encoder2 mode 4x chB
  int chA = digitalRead(corresp[2]);
  int chB = digitalRead(encoder_2_int2);
  if ((!chA & !chB) | (chA & chB)) {
    encoder_2_position++;
  }
  else {
    encoder_2_position--;
  }
}
//Encoder on INT3
void encoder_3_change_m1() { //encoder3 mode 1x
  int chB = digitalRead(encoder_3_int2);
  if (!chB) {
    encoder_3_position++;
  }
  else {
    encoder_3_position--;
  }
}
void encoder_3_change_m2() { //encoder3 mode 2x
  int chB = digitalRead(encoder_3_int2);
  int chA = digitalRead(corresp[3]);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_3_position++;
  }
  else {
    encoder_3_position--;
  }
}
void encoder_change_m4_A3() { //encoder3 mode 4x chA
  int chA = digitalRead(corresp[3]);
  int chB = digitalRead(encoder_3_int2);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_3_position++;
  }
  else {
    encoder_3_position--;
  }
}
void encoder_change_m4_B3() { //encoder3 mode 4x chB
  int chA = digitalRead(corresp[3]);
  int chB = digitalRead(encoder_3_int2);
  if ((!chA & !chB) | (chA & chB)) {
    encoder_3_position++;
  }
  else {
    encoder_3_position--;
  }
}
//Encoder on INT4
void encoder_4_change_m1() { //encoder4 mode 1x
  int chB = digitalRead(encoder_4_int2);
  if (!chB) {
    encoder_4_position++;
  }
  else {
    encoder_4_position--;
  }
}
void encoder_4_change_m2() { //encoder4 mode 2x
  int chB = digitalRead(encoder_4_int2);
  int chA = digitalRead(corresp[4]);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_4_position++;
  }
  else {
    encoder_4_position--;
  }
}
void encoder_change_m4_A4() { //encoder4 mode 4x chA
  int chA = digitalRead(corresp[4]);
  int chB = digitalRead(encoder_4_int2);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_4_position++;
  }
  else {
    encoder_4_position--;
  }
}
void encoder_change_m4_B4() { //encoder4 mode 4x chB
  int chA = digitalRead(corresp[4]);
  int chB = digitalRead(encoder_4_int2);
  if ((!chA & !chB) | (chA & chB)) {
    encoder_4_position++;
  }
  else {
    encoder_4_position--;
  }
}
//Encoder on INT5
void encoder_5_change_m1() { //encoder5 mode 1x
  int chB = digitalRead(encoder_5_int2);
  if (!chB) {
    encoder_5_position++;
  }
  else {
    encoder_5_position--;
  }
}
void encoder_5_change_m2() { //encoder5 mode 2x
  int chB = digitalRead(encoder_5_int2);
  int chA = digitalRead(corresp[5]);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_5_position++;
  }
  else {
    encoder_5_position--;
  }
}
void encoder_change_m4_A5() { //encoder5 mode 4x chA
  int chA = digitalRead(corresp[5]);
  int chB = digitalRead(encoder_5_int2);
  if ((chA & !chB) | (!chA & chB)) {
    encoder_5_position++;
  }
  else {
    encoder_5_position--;
  }
}
void encoder_change_m4_B5() { //encoder5 mode 4x chB
  int chA = digitalRead(corresp[5]);
  int chB = digitalRead(encoder_5_int2);
  if ((!chA & !chB) | (chA & chB)) {
    encoder_5_position++;
  }
  else {
    encoder_5_position--;
  }
}

/**************************************/
// Generic interrupt counter functions//
/**************************************/
//Counter on INT0
void counter_0_change() { //counter 0
  counter_0++;
}
//Counter on INT1
void counter_1_change() { //counter 1
  counter_1++;
}
//Counter on INT2
void counter_2_change() { //counter 2
  counter_2++;
}
//Counter on INT3
void counter_3_change() { //counter 3
  counter_3++;
}
//Counter on INT4
void counter_4_change() { //counter 4
  counter_4++;
}
//Counter on INT5
void counter_5_change() { //counter 5
  counter_5++;
}

//
///**************************************/
//// Acquisition with MPU6050
///**************************************/
//void get_data_MPU6050(){
//
//    if (!dmpReady) return;
//
//    // reset interrupt flag and get INT_STATUS byte
//    mpuInterrupt = false;
//    mpuIntStatus = mpu.getIntStatus();
//
//    fifoCount = mpu.getFIFOCount();
//
//    // check for overflow (this should never happen unless our code is too inefficient)
//    if ((mpuIntStatus & 0x10) || fifoCount == 1024 || fifoCount > packetSize ) {
//        mpu.resetFIFO();
//    } else if (mpuIntStatus & 0x02) {
//        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//        mpu.getFIFOBytes(fifoBuffer, packetSize);
//        fifoCount -= packetSize;
//
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            mpu.dmpGetGyro(&w, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//
//            if (acq_launched == true){
//              ypr[0]=ypr[0]* 180/M_PI;
//              ypr[1]=ypr[1]* 180/M_PI;
//              ypr[2]=ypr[2]* 180/M_PI;
//              data[0]=int(ypr[0]*100) ;
//              data[1]=int(ypr[1]*100);
//              data[2]=int(ypr[2]*100);
//              data[3]=aa.x;
//              data[4]=aa.y;
//              data[5]=aa.z;
//              data[6]=w.x;
//              data[7]=w.y;
//              data[8]=w.z;
//            }
//    }
//}
