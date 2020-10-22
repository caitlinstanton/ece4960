/******************************************************************************
  MotorTest.ino
  Serial Controlled Motor Driver
  Marshall Taylor @ SparkFun Electronics
  Sept 15, 2016
  https://github.com/sparkfun/Serial_Controlled_Motor_Driver
  https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library
  Resources:
  Uses Wire.h for i2c operation
  Uses SPI.h for SPI operation
  Development environment specifics:
  Arduino IDE 1.6.7
  Teensy loader 1.27
  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/
//This example steps through all motor positions moving them forward, then backwards.
//To use, connect a redboard to the user port, as many slaves as desired on the expansion
//port, and run the sketch.
//
// Notes:
//    While using SPI, the defualt LEDPIN will not toggle
//    This steps through all 34 motor positions, which takes a few seconds to loop.

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

#define LEDPIN 13

SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
  ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
  ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif

float alpha = 0.3;
unsigned long t0; // start time
unsigned long dt = 0; // change in time

float pitchAcc;
float pitchAccLPF;
float prevPitchAcc = 0;
float pitchGyr = 0;
float pitchFusion;
float prevPitchFusion;

float rollAcc;
float rollAccLPF;
float prevRollAcc = 0;
float rollGyr = 0;
float rollFusion;
float prevRollFusion;

float yawGyr = 0;
float xMag;
float yMag;
float yawMag;

void setup()
{
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);

  Serial.println("Starting sketch.");

  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;
  //myMotorDriver.settings.commInterface = SPI_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for slaves before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  // motor 1 inversion so that foward is the same for both motors
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1

  while ( myMotorDriver.busy() );
  myMotorDriver.enable();

  #ifdef USE_SPI
    SPI_PORT.begin();
#else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
#endif
  
  bool initialized = false;
  while( !initialized ){

#ifdef USE_SPI
    myICM.begin( CS_PIN, SPI_PORT ); 
#else
    myICM.begin( WIRE_PORT, AD0_VAL );
#endif

    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }

  Serial.println("setup complete");

}

int motorVal = 0; 
int counter = 0;
bool increase = true;

void forward() {
  myMotorDriver.setDrive(0,1,100); //drive right motor
  myMotorDriver.setDrive(1,1,125); //drive left motor
}

void backward() {
  myMotorDriver.setDrive(0,0,100); //drive right motor
  myMotorDriver.setDrive(1,0,125); //drive left motor
}

void turnRight(int speed) {
  myMotorDriver.setDrive(0,1,speed); //drive right motor
  myMotorDriver.setDrive(1,0,speed); //drive left motor
}

void stopWheels() {
  myMotorDriver.setDrive(0,1,0); //drive right motor
  myMotorDriver.setDrive(1,1,0); //drive left motor
}

void brakeForwards() {
  myMotorDriver.setDrive(0,0,255); //drive right motor
  myMotorDriver.setDrive(1,0,255); //drive left motor
}

float getPitchAcc(float x, float z) {
  return atan2(x,z)*180/M_PI;
}

float getRollAcc(float y, float z) {
  return atan2(y,z)*180/M_PI;
}

float getPitchGyr(float prev, float x) {
  return prev - x * (float)dt/1000000;  
}

float getRollGyr(float prev, float y) {
  return prev - y * (float)dt/1000000;
}

float getYawGyr(float prev, float z) {
  return prev - z * (float)dt/1000000;
}

float getYawMag(float xMag, float yMag) {
  return atan2(yMag, xMag) * 180/M_PI;
}

float applyLPF(float curr, float prev, float alpha){
  return alpha * curr + (1 - alpha) * prev;
}

void loop()
{
  if( myICM.dataReady() ){
    t0 = micros(); // start time
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'

    //***** Operate the Motor Driver *****//
    //  It uses .setDrive( motorName, direction, level ) to drive the motors.
    if(counter>0)
    {
      // if its time for the next motor value, incease/decrease it
      if(increase)
      {
        motorVal++;
      }
      else
      {
        motorVal--;
      }
      counter=0;
      if(motorVal == 255 || motorVal == 0)
      {
          increase = !increase; //switch direction if cant increase anymore
      }
    }
 
    // PITCH
    pitchAcc = getPitchAcc(myICM.accX(),myICM.accZ());
    pitchAccLPF = applyLPF(pitchAcc, prevPitchAcc, 0.2);
    prevPitchAcc = pitchAccLPF;
    pitchGyr = getPitchGyr(pitchGyr, myICM.gyrY());
    pitchFusion = (prevPitchFusion + pitchGyr * dt/1000000) * (1-alpha) + pitchAccLPF * alpha;
    prevPitchFusion = pitchFusion; 

    // ROLL
    rollAcc = -getRollAcc(myICM.accY(),myICM.accZ()); //flipped sign to be consistent with gyroscope data
    rollAccLPF = applyLPF(rollAcc, prevRollAcc, alpha);
    prevRollAcc = rollAccLPF;
    rollGyr = getRollGyr(rollGyr, myICM.gyrX());
    rollFusion = (prevRollFusion + rollGyr * dt/1000000) * (1-alpha) + rollAccLPF * alpha;
    prevRollFusion = rollFusion;

    // YAW
    yawGyr = getYawGyr(yawGyr, myICM.gyrZ());
    xMag = myICM.magX()*cos(pitchGyr*M_PI/180)-myICM.magZ()*sin(pitchGyr*M_PI/180);
    yMag = myICM.magY()*sin(pitchGyr*M_PI/180)*sin(rollGyr*M_PI/180)-myICM.magY()*cos(rollGyr*M_PI/180)+myICM.magZ()*cos(pitchGyr*M_PI/180)*cos(rollGyr*M_PI/180);
    yawMag = atan2((myICM.magX()*cos(pitchFusion) + myICM.magZ()*sin(pitchFusion)), (myICM.magY()*cos(rollFusion) + myICM.magZ()*sin(rollFusion)));
  
    //set motors
    Serial.print(motorVal);
    Serial.print("\t");
    Serial.print(yawMag);
    Serial.println("");
    turnRight(motorVal);
    counter++; //increment counts
    
    dt = micros() - t0;    
    delay(500);
  }else{
    Serial.println("Waiting for data");
    delay(500);
  }
}
