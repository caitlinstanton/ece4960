#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "SparkFun_VCNL4040_Arduino_Library.h"

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.
VCNL4040 proximitySensor;
SFEVL53L1X distanceSensor;

//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

void setup(void)
{
  Serial.begin(115200);
  //motor setup
  myMotorDriver.settings.commInterface = I2C_MODE;
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D
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
  // motor 1 inversion so that foward is the same for both motors
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1
  while ( myMotorDriver.busy() );
  myMotorDriver.enable();

  // sensor setup  
  Wire.begin();
  if (proximitySensor.begin() == false)
  {
    Serial.println("Device not found. Please check wiring.");
    while (1); //Freeze!
  }
  //Serial.println("VL53L1X Qwiic Test");
  //VL53L1_SetInterMeasurementPeriodMilliSeconds(&VL53L1Dev, 1000 );
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  distanceSensor.setOffset(41);
  distanceSensor.setTimingBudgetInMs(35);
  distanceSensor.setIntermeasurementPeriod(5);
  distanceSensor.setDistanceModeShort();
  distanceSensor.startRanging(); //just continue ranging the whole time to save time turning it on/off
}

void forward() {
  myMotorDriver.setDrive(0,1,100); //drive right motor
  myMotorDriver.setDrive(1,1,125); //drive left motor
}

void backward() {
  myMotorDriver.setDrive(0,0,100); //drive right motor
  myMotorDriver.setDrive(1,0,125); //drive left motor
}

void turnRight() {
  myMotorDriver.setDrive(0,1,200); //drive right motor
  myMotorDriver.setDrive(1,1,200); //drive left motor
}

void stopWheels() {
  myMotorDriver.setDrive(0,1,0); //drive right motor
  myMotorDriver.setDrive(1,1,0); //drive left motor
}

void brakeHard() {
  myMotorDriver.setDrive(0,1,200);//reverse for a touch
  myMotorDriver.setDrive(1,0,200);
  delay(200);
  myMotorDriver.setDrive(0,0,0);
  myMotorDriver.setDrive(1,0,0);
}

void loop(void)
{
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the ToF sensor
  unsigned int proxValue = proximitySensor.getProximity();  //Get result from prox sensor
  distanceSensor.clearInterrupt();

  Serial.print("Distance(mm): ");
  Serial.print(distance);
  Serial.print(", ");
  Serial.print(distanceSensor.getRangeStatus());
  Serial.print("\tProx: ");
  Serial.print(proxValue);
  Serial.print("\n");
  
  byte rangeStatus = distanceSensor.getRangeStatus();
  if(rangeStatus==0)  //only act if sensor reading was good
  {
    if((distance==0 || distance>300) && proxValue<20) //0 if there is no object nearby
    {
      forward();
    }
    else
    {
      brakeHard();
      //turnRight();
      delay(400);
    }
  }
}
