
/*
  04/19/2020 Kerry Edited
  AndroidArtemisBleUartClient
  https://github.com/kerryeven/AndroidArtemisBleUartClient

  June 2020 Ta-Weh Yeh Edited
  https://github.com/TaWeiYeh/ArtemisBleApp

  August 2020 Alex Coy Tailor for ECE 4960
*/
/********************************************************************************************************************
                 INCLUDES
 *******************************************************************************************************************/
#include "BLE_example.h"
#include "commands.h"
#include "related_funcs.h"

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
#include "SparkFun_VL53L1X.h" 
#include "SparkFun_VCNL4040_Arduino_Library.h"
#include "ICM_20948.h"  // \\\Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"

#include <PID_v1.h> //PID library

// maximum length of reply / data message
#define MAXREPLY 100
#define TODO_VAL 0
// buffer to reply to client
uint8_t val[MAXREPLY];
uint8_t *val_data = &val[2]; // start of optional data area
uint8_t *val_len = &val[1];  // store length of optional data
int packet_count = 0;

/********************************************************************************************************************
                  OBJECTS
  *******************************************************************************************************************/

/********************************************************************************************************************
                GLOBAL VARIABLES
 *******************************************************************************************************************/
String s_Rev = "Rev 1.0";
String s_Rcvd = "100"; //KHE declare extern in BLE_example_funcs.cpp to accept messages, if 100 don't bother checking case statements
uint16_t l_Rcvd = 0;
uint8_t *m_Rcvd = NULL;
String s_AdvName = "MyRobot"; //KHE 2 0 TOTAL CHARACHTERS ONLY!!  any more will be dropped

cmd_t empty_cmd = {NOT_A_COMMAND, 1, {0}};
cmd_t *cmd = &empty_cmd;
cmd_t *res_cmd = &empty_cmd;
bt_debug_msg_t *bt_debug_head = NULL;
bt_debug_msg_t *bt_debug_tail = NULL;

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

present_t presentSensors = {
    .motorDriver = 0,
    .ToF_sensor = 0,
    .prox_sensor = 0,
    .IMU = 0};
int bytestream_active = 0;

//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.
VCNL4040 proximitySensor;
SFEVL53L1X distanceSensor;

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
float prevYawGyr;
float xMag;
float yMag;
float yawMag;

boolean motorRamp = false;
unsigned char motorVal = 1; 
int counterRamp = 0;
int counterTransmit = 0;
bool increase = true;

double Setpoint, Input, Output; // for PID control
double Kp=10, Ki=0, Kd=0; //CHANGE THESE CONSTANTS FOR PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/*************************************************************************************************/
/*!
     \fn     setup

     \brief  Arduino setup function.  Set up the board BLE and GPIO - BLE first...

     \param  none

     \called Arduino startup

     \return None.
 */
/*************************************************************************************************/
void setup()
{
    Serial.begin(115200);
    delay(1000);

    while(!SERIAL_PORT){};

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

#ifdef BLE_SHOW_DATA
//SERIAL_PORT.begin(115200);
//delay(1000);
//SERIAL_PORT.printf("Viper. Compiled: %s\n" , __TIME__);
#endif

#ifdef AM_DEBUG_PRINTF
    //
    // Enable printing to the console.
    //
    enable_print_interface();
#endif

    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    Serial.print("Revision = ");
    Serial.print(s_Rev);
    Serial.printf("  ECE 4960 Robot Compiled: %s   %s\n", __DATE__, __TIME__);
    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    analogWriteResolution(16); //Set AnalogWrite resolution to 16 bit = 0 - 65535 (but make max 64k or trouble)

    /********************************************************************************************************************
                    Set Advertising name:  uses global string s_AdvName set above.
     *******************************************************************************************************************/
    set_Adv_Name(); //BLE_example_funcs.cpp near end of file, fn declared extern in BLE_example.h

    /********************************************************************************************************************
                     Boot the radio
                      SDK/third_party/exactle/sw/hci/apollo3/hci_drv_apollo3.c
                      = huge program to handle the ble radio stuff in this file
     *******************************************************************************************************************/
    HciDrvRadioBoot(0);

    /************************************************************************************************
          Initialize the main ExactLE stack: BLE_example_funcs.cpp
          - One time timer
          - timer for handler
          - dynamic memory buffer
          - security
          - HCI host conroller interface
          - DM device manager software
          - L2CAP data transfer management
          - ATT - Low Energy handlers
          - SMP - Low Energy security
          - APP - application handlers..global settings..etc
          - NUS - nordic location services

     ************************************************************************************************/
    exactle_stack_init();

    /*************************************************************************************************
        Set the power level to it's maximum of 15 decimal...defined in hci_drv_apollo3.h as 0xF
        needs to come after the HCI stack is initialized in previous line
          - poss. levels = 0x03=-20,0x04=-10,0x05=-5,0x08=0,0x0F=4 but have to use definitions, not these ints
            extremes make a difference of about 10 at 1 foot.
     ************************************************************************************************/
    HciVsA3_SetRfPowerLevelEx(TX_POWER_LEVEL_PLUS_3P0_dBm); //= 15 decimal = max power WORKS..default = 0

    /*************************************************************************************************
        Start the "Amdtp" (AmbiqMicro Data Transfer Protocol) profile. Function in amdtp_main.c

         Register for stack callbacks
         - Register callback with DM for scan and advertising events with security
         - Register callback with Connection Manager with client id
         - Register callback with ATT low energy handlers
         - Register callback with ATT low enerty connection handlers
         - Register callback with ATT CCC = client charachteristic configuration array
         - Register for app framework discovery callbacks
         - Initialize attribute server database
         - Reset the device

     ************************************************************************************************/
    AmdtpStart();

    /*************************************************************************************************
       On first boot after upload and boot from battery, pwm on pin 14 not working
        need to reset nano board several times with battery power applied to get
        working.  Delay 5 seconds works..haven't tried lesser values.
     ************************************************************************************************/
    //delay(5000);

    /************************************************************************************************
        Arduino device GPIO control setup.
          Place after board BLE setup stuff happens.  ie.:
            could not get A14 to PWM untill I moved the set_stop() call from the
            beginning of setup to this location...then works great.
     ************************************************************************************************/

    pinMode(LED_BUILTIN, OUTPUT);

    //Set a starting point...for motors, servo, and LED_BUILTIN
    //delay(1000);

    // Bluetooth would start after blinking
    for (int i = 0; i < 20; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }

    //setupwdt();

    pinMode(blinkPin, OUTPUT);
    digitalWrite(blinkPin, LOW);

    // Configure the watchdog.
    //setupTimerA(myTimer, 31); // timerNum, period - //moved to BLE_example_funcs.cpp scheduler_timer_init
    setupWdt();
    am_hal_wdt_init(&g_sWatchdogConfig);
    //NVIC_EnableIRQ(CTIMER_IRQn); // Enable CTIMER interrupt in nested vector interrupt controller.
    NVIC_EnableIRQ(WDT_IRQn); // Enable WDT interrupt in nested vector interrupt controller.

    uint8_t a = 0;
    m_Rcvd = &a;
    //Serial.printf("Size of command: %d", sizeof(cmd_t));
    am_hal_interrupt_master_enable();
    //interrupts(); // Enable interrupt operation. Equivalent to am_hal_rtc_int_enable().
    //am_hal_wdt_start();
    //am_hal_wdt_int_enable(); - freezes boot

} /*** END setup FCN ***/

void forward() {
  myMotorDriver.setDrive(0,1,100); //drive right motor
  myMotorDriver.setDrive(1,1,125); //drive left motor
}

void backward() {
  myMotorDriver.setDrive(0,0,100); //drive right motor
  myMotorDriver.setDrive(1,0,125); //drive left motor
}

void turnRight(unsigned char speed) {
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
      myICM.getAGMT();  
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
    
      Input = myICM.gyrZ();
      Setpoint = 50;
      myPID.Compute(); //compute Output for motors
      if(Output>190) motorVal = 190;
      else motorVal = Output;
      myMotorDriver.setDrive( 1, 1, motorVal); 
      myMotorDriver.setDrive( 0, 1, motorVal);
  
  
      Serial.print("MotorValue:");
      Serial.print(motorVal);
      Serial.print(" ");
      Serial.print("GyroData:");
      Serial.println(Input);
    }else{
      Serial.println("Waiting for data");
      delay(500);
    }
    
    counterTransmit++; 
    //Serial.println("Loop...."); //KHE Loops constantly....no delays

    if (l_Rcvd > 1) //Check if we have a new message from amdtps_main.c through BLE_example_funcs.cpp
    {

        cmd = (cmd_t *)m_Rcvd;
        /*
        Serial.print("Message Buffer: ");
        for (int i = 0; i < l_Rcvd; i++)
            Serial.printf("%d ", m_Rcvd[i]);
        Serial.println();
        Serial.printf("Got command: 0x%x Length: 0x%x Data: ", cmd->command_type, cmd->length);

        for (int i = 0; i < cmd->length; i++)
        {
            Serial.printf("0x%x ", cmd->data[i]);
        }
        Serial.println();
        */

        switch (cmd->command_type)
        {
        case SET_MOTORS:

            Serial.println("Placeholder: Set Motors");
            break;
        case GET_MOTORS:

            Serial.println("Placeholder: Set Motors");
            //amdtpsSendData((uint8_t *)res_cmd, *val_len);
            break;
        case SER_RX:
            Serial.println("Got a serial message");
            pushMessage((char *)&cmd->data, cmd->length);
//            if (!kp_recv) {
//              kp = (float)pullMessage();
//              Serial.print("kp: ");
//              Serial.println(kp);
//            }
            break;
        case REQ_FLOAT:
            Serial.println("Going to send a float");
            //TODO: Put a float (perhaps pi) into a command response and send it.
            res_cmd->command_type = GIVE_FLOAT;     //set command type as GIVE_FLOAT
            res_cmd->length=6;                      
            ((float *)(res_cmd->data))[0] = 3.14f;  //put a float into data to send
            amdtpsSendData((uint8_t *)res_cmd, 6);  //2 bytes for type and length, 4 bytes of data
            break;
        case PING:
            Serial.println("Ping Pong");
            cmd->command_type = PONG;
            amdtpsSendData(m_Rcvd, l_Rcvd);
            break;
        case START_BYTESTREAM_TX:
            bytestream_active = (int)cmd->data[0];
            Serial.printf("Start bytestream with active %d \n", bytestream_active);
            ((uint32_t *)res_cmd->data)[0] = 0;
            bytestream_active = 1;
            motorRamp = true;
            
            break;
        case STOP_BYTESTREAM_TX:
            bytestream_active = 0;
            break;
        default:
            Serial.printf("Unsupported Command 0x%x \n", cmd->command_type);
            break;
        }

        l_Rcvd = 0;
        am_hal_wdt_restart();
        free(m_Rcvd);
    } //End if s_Rcvd != 100
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '7'))
    {
        s_Rcvd[0] = 0;
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.printf("Connected, length was %d", l_Rcvd);
    }
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '8'))
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("disconnected");
        //Decimal value of D for Disconnect
        //Serial.println("got disconnect from case in ino file - set_Stop");
        digitalWrite(LED_BUILTIN, LOW);
        //amdtps_conn_close();
        DmDevReset();
    }

    if (availableMessage())
    {
        Serial.println("Bluetooth Message:");
        Serial.println(pullMessage());
        printOverBluetooth("Message Received.");
    }

    if (bytestream_active)
    {
        if(counterTransmit>=20) //no need to send ALL the data
        {
          res_cmd->command_type = BYTESTREAM_TX;  //set command type to bytestream transmit
          res_cmd->length = 11;                    //length doesn't matter since the handler will take care of this
          //TODO: Put an example of a 32-bit integer and a 64-bit integer
          //for the stream. Be sure to add a corresponding case in the
          //python program.
          //Serial.printf("Stream %d \n", bytestream_active);
       
          // pack up data to send
          unsigned long t=micros(); //send current time for x axis
          memcpy(res_cmd->data, &t, 4); 
          memcpy(res_cmd->data+4, &motorVal, 1);
          memcpy(res_cmd->data+5, &yawGyr, 4);
          amdtpsSendData((uint8_t *)res_cmd, 11);  //2 bytes for type and length, 14 bytes of data
          counterTransmit = 0;
        }
      //Print time
      unsigned long t = micros();
      Serial.printf("Packet %d sent at %d micro seconds \n", packet_count, t);
      packet_count++;
    }
    trigger_timers();
    dt = micros() - t0;  
} //END LOOP
