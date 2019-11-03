

// =======================================================================================
//                 SHAD-0W :  Small Handheld Arduino D-0 Wand
// =======================================================================================
//                          Last Revised Date: 11/01/2019
//                                Version: 0.4
//                             Written By: jonhaag
//               Inspired by the SHADOW by KnightShade & PADAWAN by danf
//                    Movement code based on work by MrBaddeley
//                 Designed to work on v1.0 of MrBaddeley's D-0 files
//                            
// =======================================================================================
//                              Revision History/Notes
// =======================================================================================
//v0.4 - Re-enabled sound & headBar automation; moved drive system on/off to L2+O/X
//v0.3 - Disabled headBar & soundboard start up; added dome automation on/off with L1+O/X
//
//
//
// ======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it .
//         This program is distributed in the hope that it will be useful,
//         but WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// =======================================================================================
//   Note: You will need a Arduino Mega 2560 to run this sketch
//
//   This is written to be a UNIVERSAL Sketch - supporting multiple controller options
//      - Single PS3 Move Navigation
//      - Pair of PS3 Move Navigation - to be added later
//
//   PS3 Bluetooth library - developed by Kristian Lauszus (kristianl@tkjelectronics.com)
//
// =======================================================================================
//
// ---------------------------------------------------------------------------------------
//                          User Settings
// ---------------------------------------------------------------------------------------

//Primary Controller
String PS3MoveNavigatonPrimaryMAC = "04:76:6E:5A:9A:5B"; //If using multiple controlers, designate a primary

byte joystickDeadZoneRange = 10;  // For controllers that centering problems, use the lowest number with no drift

int steeringNeutral = 0;        // Move this by one or two to set the center point for the steering servo

int drive1Neutral = 330;
int drive2Neutral = 325; // Move this by one or two to set the center point for the drive ESC

//#define TEST_CONROLLER         //Support coming soon
//#define SHADOW_DEBUG           //uncomment this for console DEBUG output
#define SHADOW_VERBOSE           //uncomment this for console VERBOSE output


#define HEAD_BAR_SERVO_MIN 20
#define HEAD_BAR_SERVO_MAX 160


// ---------------------------------------------------------------------------------------
//                          Sound Control Settings
// ---------------------------------------------------------------------------------------




// ---------------------------------------------------------------------------------------
//                          Libraries
// ---------------------------------------------------------------------------------------
#include <PS3BT.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include "ServoEaser.h"
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>


// ---------------------------------------------------------------------------------------
//                          Variables
// ---------------------------------------------------------------------------------------

long previousMillis = millis();
long currentMillis = millis();
int serialLatency = 25;          // This is a delay factor in ms to prevent queueing of the Serial data.
                                 // 25ms seems approprate for HardwareSerial, values of 50ms or larger
                                 // are needed for Softare Emulation

///////Setup for USB and Bluetooth Devices////////////////////////////
USB Usb;
BTD Btd(&Usb);                   // You have to create the Bluetooth Dongle instance like so
PS3BT *PS3Nav = new PS3BT(&Btd);
PS3BT *PS3Nav2 = new PS3BT(&Btd);
//Used for PS3 Fault Detection
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;

boolean firstMessage = true;
String output = "";

boolean isDriveMotorStopped = true;

boolean isPS3NavigatonInitialized = false;
boolean isSecondaryPS3NavigatonInitialized = false;

byte vol = 0;                   // 0 = full volume, 255 off for MP3Trigger, the MDFly use a 0-31 vol range
boolean isStickEnabled = true;

byte action = 0;
unsigned long DriveMillis = 0;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

int servoFrameMillis = 20;

//Servo headBarServo; 
//  
//ServoEaser headBarServoEaser;

const int HEAD_BAR_CENTER = 290;
const int HEAD_BAR_FRONT = HEAD_BAR_CENTER - 100;
const int HEAD_BAR_BACK = HEAD_BAR_CENTER + 100;

const int NOD_BAR_CENTER = 340;
const int NOD_BAR_MAX = NOD_BAR_CENTER + 100;
const int NOD_BAR_MIN = NOD_BAR_CENTER - 100;

Servo servo1; 
Servo servo2;

ServoEaser servoEaser1;
ServoEaser servoEaser2;

const int Headservo1Pin = 2;
const int Headservo2Pin = 5;

unsigned long lastMillis;

int Wheel1;
int Wheel2;

int16_t WheelServo1; //LeftWheel
int16_t WheelServo2; //RightWheel

int Targetspeed1 = 0; //variable to store target speed
int Targetspeed2 = 0; //variable to store target speed
int CurrentWheel1 = 330;  // variable to store current speed for wheel 1
int CurrentWheel2 = 340;  // variable to store current speed for wheel 2

int WHEEL1_CENTER = 330;
int WHEEL2_CENTER = 340;
int DEADZONE = 20;
const int TURN_DEADZONE = 20;

int SpeedChange = 5;  // variable to store speed step change (lower, more lag / smoother)
int DrivePeriod = 10; //increase speed every x milliseconds, (higher, more lag / smoother)
unsigned long Drivetime_now = 0; // Drivetime current, 

int DirectionState = 0; // 0 for Stationary, 1 for forward, 2 for backwards
int SteerState = 0; // 0 for Stationary, 1 for left, 2 for right

int headValue; //will hold head bar values
int HeadState = 0;
int HeadTurn1;
int HeadTurn2;

int Wheel1pos = 0;    // variable to store the Wheel1 speed 
int Wheel2pos = 0;    // variable to store the Wheel2 speed
float Steerpos = 0; //Variable to adjust the steering
float Steeradjust = 0;

// =======================================================================================
//                          Music Maker Shield Setup
// =======================================================================================

#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer = 
  // create breakout-example object!
  //Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
  // create shield-example object!
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);

// =======================================================================================
//                          Main Program
// =======================================================================================

void setup()
{
  //Debug Serial for use with USB Debugging
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1)
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nBluetooth Library Started"));
  output.reserve(200); // Reserve 200 bytes for the output string

  //Setup for PS3
  PS3Nav->attachOnInit(onInitPS3); // onInit() is called upon a new connection - you can call the function whatever you like
  PS3Nav2->attachOnInit(onInitPS3Nav2);

  //Startup Music Player
  // initialise the music player
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
    // while (1);
  }
  Serial.println(F("VS1053 found"));

    if (! musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT))
    Serial.println(F("DREQ pin is not an interrupt pin"));
  
  musicPlayer.setVolume(5,5);
//  musicPlayer.playFullFile("/track001.mp3");
  //musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
    if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
   // while (1);  // don't do anything more
  }
  Serial.println("SD OK!");
  printDirectory(SD.open("/"), 0);

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  Serial.print(F("\r\nPWM started"));
   
  Serial.print(F("\r\nD-0 Drive Running"));

  delay(10);

  //Initialize the Head Bar servo and move it into upright position if not already there
  //headBarServo.attach(headBarServoPin);
  //headBarServoEaser.begin(headBarServo, servoFrameMillis);
  //headBarServoEaser.easeTo(90,2000);

  servo1.attach( Headservo1Pin );
  servo2.attach( Headservo2Pin );

  servoEaser1.begin( servo1, servoFrameMillis );
  servoEaser2.begin( servo2, servoFrameMillis );

  servoEaser1.easeTo( 90, 2000);
  servoEaser2.easeTo( 90, 2000);
   
  HeadState =0; 

  pwm.setPWM(2, 0, HEAD_BAR_CENTER);
  pwm.setPWM(3, 0, NOD_BAR_CENTER);

  musicPlayer.playFullFile("track000.mp3");
}

boolean readUSB()
{
  //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  Usb.Task();
  if (PS3Nav->PS3NavigationConnected ) Usb.Task();
  if (PS3Nav2->PS3NavigationConnected ) Usb.Task();
  if ( criticalFaultDetect() )
  {
    //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
    flushAndroidTerminal();
    return false;
  }
  return true;
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);


}

void loop()
{
    
  //Useful to enable with serial console when having controller issues.
#ifdef TEST_CONROLLER
  testPS3Controller();
#endif

  //LOOP through functions from highest to lowest priority.

  if ( !readUSB() )
  {
    //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
    return;
  }
//  headBarServoEaser.update();
  
servoEaser1.update();
servoEaser2.update();


if (HeadState == 0) {
  if (servoEaser1.hasArrived() ) {
      lastMillis = millis();
      servoEaser1.easeTo( 90, 2000);
  }
    if (servoEaser1.hasArrived() ) {
      lastMillis = millis();
      servoEaser2.easeTo( 90, 2000);
  }
}

if (HeadState == 1) {
  if (servoEaser2.hasArrived() ) {
 lastMillis = millis();
    int angle    = random(30,150);
    int duration = random(1000,1500); 
    servoEaser2.easeTo( angle, duration );
}

if (servoEaser1.hasArrived() ) {
 lastMillis = millis();
    int angle    = random(70,110);
    int duration = random(1000,1500); 
    servoEaser1.easeTo( angle, duration );
}
}
  Drive();

  if ( !readUSB() )
  {
    //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
    return;
  }
  
  toggleSettings();
  soundControl();
  flushAndroidTerminal();
}

void onInitPS3()
{
  String btAddress = getLastConnectedBtMAC();
  PS3Nav->setLedOn(LED1);
  isPS3NavigatonInitialized = true;
  badPS3Data = 0;
#ifdef SHADOW_DEBUG
  output += "\r\nBT Address of Last connected Device when Primary PS3 Connected: ";
  output += btAddress;
  if (btAddress == PS3MoveNavigatonPrimaryMAC)
  {
    output += "\r\nWe have our primary controller connected.\r\n";
  }
  else
  {
    output += "\r\nWe have a controller connected, but it is not designated as \"primary\".\r\n";
  }
#endif
}

void onInitPS3Nav2()
{
  String btAddress = getLastConnectedBtMAC();
  PS3Nav2->setLedOn(LED1);
  isSecondaryPS3NavigatonInitialized = true;
  badPS3Data = 0;
  if (btAddress == PS3MoveNavigatonPrimaryMAC) swapPS3NavControllers();
#ifdef SHADOW_DEBUG
  output += "\r\nBT Address of Last connected Device when Secondary PS3 Connected: ";
  output += btAddress;
  if (btAddress == PS3MoveNavigatonPrimaryMAC)
  {
    output += "\r\nWe have our primary controller connecting out of order.  Swapping locations\r\n";
  }
  else
  {
    output += "\r\nWe have a secondary controller connected.\r\n";
  }
#endif
}

String getLastConnectedBtMAC()
{
  String btAddress = "";
  for (int8_t i = 5; i > 0; i--)
  {
    if (Btd.disc_bdaddr[i] < 0x10)
    {
      btAddress += "0";
    }
    btAddress += String(Btd.disc_bdaddr[i], HEX);
    btAddress += (":");
  }
  btAddress += String(Btd.disc_bdaddr[0], HEX);
  btAddress.toUpperCase();
  return btAddress;
}

void swapPS3NavControllers()
{
  PS3BT* temp = PS3Nav;
  PS3Nav = PS3Nav2;
  PS3Nav2 = temp;
  //Correct the status for Initialization
  boolean tempStatus = isPS3NavigatonInitialized;
  isPS3NavigatonInitialized = isSecondaryPS3NavigatonInitialized;
  isSecondaryPS3NavigatonInitialized = tempStatus;
  //Must relink the correct onInit calls
  PS3Nav->attachOnInit(onInitPS3);
  PS3Nav2->attachOnInit(onInitPS3Nav2);
}

void flushAndroidTerminal()
{
  if (output != "")
  {
    if (Serial) Serial.println(output);
    output = ""; // Reset output string
  }
}

// =======================================================================================
// //////////////////////////Process PS3 Controller Fault Detection///////////////////////
// =======================================================================================
boolean criticalFaultDetect()
{
  if (PS3Nav->PS3NavigationConnected || PS3Nav->PS3Connected)
  {
    lastMsgTime = PS3Nav->getLastMessageTime();
    currentTime = millis();
    if ( currentTime >= lastMsgTime)
    {
      msgLagTime = currentTime - lastMsgTime;
    } else
    {
#ifdef SHADOW_DEBUG
      output += "Waiting for PS3Nav Controller Data\r\n";
#endif
      badPS3Data++;
      msgLagTime = 0;
    }

    if (msgLagTime > 100 && !isDriveMotorStopped)
    {
#ifdef SHADOW_DEBUG
      output += "It has been 100ms since we heard from the PS3 Controller\r\n";
      output += "Shutting down motors, and watching for a new PS3 message\r\n";
#endif

      WheelServo1 = 4096;
      WheelServo2 = 4096; 
      isDriveMotorStopped = true;
      return true;
    }
    if ( msgLagTime > 30000 )
    {
#ifdef SHADOW_DEBUG
      output += "It has been 30s since we heard from the PS3 Controller\r\n";
      output += "msgLagTime:";
      output += msgLagTime;
      output += "  lastMsgTime:";
      output += lastMsgTime;
      output += "  millis:";
      output += millis();
      output += "\r\nDisconnecting the controller.\r\n";
#endif
      PS3Nav->disconnect();
    }

    //Check PS3 Signal Data
    if (!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged))
    {
      // We don't have good data from the controller.
      //Wait 10ms, Update USB, and try again
      delay(10);
      Usb.Task();
      if (!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged))
      {
        badPS3Data++;
#ifdef SHADOW_DEBUG
        output += "\r\nInvalid data from PS3 Controller.";
#endif
        return true;
      }
    }
    else if (badPS3Data > 0)
    {
      //output += "\r\nPS3 Controller  - Recovered from noisy connection after: ";
      //output += badPS3Data;
      badPS3Data = 0;
    }
    if ( badPS3Data > 10 )
    {
#ifdef SHADOW_DEBUG
      output += "Too much bad data coming fromo the PS3 Controller\r\n";
      output += "Disconnecting the controller.\r\n";
#endif
      PS3Nav->disconnect();
    }
  }
  else if (!isDriveMotorStopped)
  {
#ifdef SHADOW_DEBUG
    output += "No Connected Controllers were found\r\n";
    output += "Shuting downing motors, and watching for a new PS3 message\r\n";
#endif

    WheelServo1 = 4096;
    WheelServo2 = 4096; 
    isDriveMotorStopped = true;
    return true;
  }
  return false;
}
// =======================================================================================
// //////////////////////////END of PS3 Controller Fault Detection///////////////////////
// =======================================================================================


boolean ps3Drive(PS3BT* myPS3 = PS3Nav)
{
  if (isPS3NavigatonInitialized) {
    // Additional fault control.  Do NOT send additional commands if no controllers have initialized.
    if (!isStickEnabled) {
      #ifdef SHADOW_VERBOSE
      if ( abs(myPS3->getAnalogHat(LeftHatY) - 128) > joystickDeadZoneRange)
      {
        output += "Drive Stick is disabled\r\n";
      }
      #endif

      WheelServo1 = 4096;
      WheelServo2 = 4096; 
      isDriveMotorStopped = true;
      
    } else if (!myPS3->PS3NavigationConnected) {
      WheelServo1 = 4096;
      WheelServo2 = 4096; 
      isDriveMotorStopped = true;

    } else if (!(myPS3->getButtonPress(L1) || myPS3->getButtonPress(L2))) {
      int stickX = myPS3->getAnalogHat(LeftHatX);
      int stickY = myPS3->getAnalogHat(LeftHatY);

      Steerpos = map(stickX, 0, 255, -155, 155);
      Wheel1pos = map(stickY, 255, 0, 180, 490);
      Wheel2pos = map(stickY, 255, 0, 500, 190);

      //Create Steeradjust / SteerState value
      if (Steerpos > TURN_DEADZONE) {
        SteerState =1;
      }
      else if (Steerpos <-TURN_DEADZONE) {
        SteerState =2;
      }
      else if (Steerpos >-TURN_DEADZONE && Steerpos <TURN_DEADZONE){
        SteerState = 0;
      }
      
      Steeradjust = Steerpos / 155;
      
      if (Steeradjust <0){
        Steeradjust = -Steeradjust;
      
      }

      //Ramp up / Ramp down
      
      Targetspeed1 = Wheel1pos;
      Targetspeed2 = Wheel2pos;

      if(millis() > Drivetime_now + DrivePeriod){      
        Drivetime_now = millis();      
      if (Targetspeed1>CurrentWheel1) {
        CurrentWheel1=CurrentWheel1+ SpeedChange;
          }
      else if (Targetspeed1<CurrentWheel1) {
        CurrentWheel1 = CurrentWheel1 - SpeedChange;
          }
          if (Targetspeed2>CurrentWheel2) {
            CurrentWheel2=CurrentWheel2+ SpeedChange;
          }
      else if (Targetspeed2<CurrentWheel2) {
        CurrentWheel2 = CurrentWheel2 - SpeedChange;
          }
      }
      WheelServo1 = CurrentWheel1;
      WheelServo2 = CurrentWheel2;

      //kill wheel movement if centred (stops creep). 
      if (CurrentWheel1 > (WHEEL1_CENTER - DEADZONE) && CurrentWheel1 < (WHEEL1_CENTER + DEADZONE)) {
            WheelServo1 = 4096;     
          }
      if (CurrentWheel2 > (WHEEL2_CENTER - DEADZONE) && CurrentWheel2 < (WHEEL2_CENTER + DEADZONE)) {
            WheelServo2 = 4096;   
          }
      
      
      //Check directionstate and move head ready for movement. 
      if (CurrentWheel1> (WHEEL1_CENTER + DEADZONE) && CurrentWheel2< (WHEEL2_CENTER - DEADZONE)) {
        DirectionState=1;
        headValue = HEAD_BAR_FRONT;
      }
      else if (CurrentWheel1 < (WHEEL1_CENTER - DEADZONE) && CurrentWheel2> (WHEEL2_CENTER + DEADZONE)) {
        DirectionState=2;
        headValue = HEAD_BAR_BACK;
      }
      else if (CurrentWheel1 > (WHEEL1_CENTER - DEADZONE) && CurrentWheel1 < (WHEEL1_CENTER + DEADZONE) && CurrentWheel2 > (WHEEL2_CENTER - DEADZONE) && CurrentWheel2 < (WHEEL2_CENTER + DEADZONE)) {
        DirectionState=0;
        headValue = HEAD_BAR_CENTER;
      }
      
      //Steering modifier
      
      if (CurrentWheel1> (WHEEL1_CENTER - DEADZONE) && CurrentWheel1 < (WHEEL1_CENTER + DEADZONE) && CurrentWheel2 > (WHEEL2_CENTER - DEADZONE) && CurrentWheel2 < (WHEEL2_CENTER + DEADZONE) && (Steerpos > 20 || Steerpos < 0)) {
      
        WheelServo1 = CurrentWheel1+(-Steerpos / 3);
        WheelServo2 = CurrentWheel2+(-Steerpos / 3);
        
      }
      
      else if ((CurrentWheel1 < (WHEEL1_CENTER - DEADZONE) || CurrentWheel1 > (WHEEL1_CENTER + DEADZONE)) && (CurrentWheel2 < (WHEEL2_CENTER - DEADZONE) || CurrentWheel2 >(WHEEL2_CENTER + DEADZONE))){
      
    
        
      }
      
      
      //moving steering
      
      if (DirectionState ==1) {
      
      WheelServo1 = CurrentWheel1+(-Steerpos / 3);
      WheelServo2 = CurrentWheel2+(-Steerpos / 3);
        
      }
      
      if (DirectionState ==2) {
      
      WheelServo1 = CurrentWheel1+(-Steerpos / 3);
      WheelServo2 = CurrentWheel2+(-Steerpos / 3);
        
      }

      Serial.print ("stickY: ");
      Serial.print (stickY);      Serial.print ("stickX: ");
      Serial.print (stickX);
      Serial.print (" Steerpos: ");
      Serial.print (Steerpos);
      Serial.print (" WheelServo1: ");
      Serial.print (WheelServo1);     
      Serial.print (" WheelServo2: ");
      Serial.print (WheelServo2); 
      Serial.print ("\r\n");


//      if (headBarServoEaser.hasArrived() ) {
//        lastMillis = millis();
//        headBarServoEaser.easeTo(headValue,1000); 
//      }
         
      
      pwm.setPWM(0, 0, WheelServo1);
      pwm.setPWM(1, 0, WheelServo2);
      pwm.setPWM(2, 0, headValue);

      

      return true; //we sent a drive command
      
    } else if (myPS3->getButtonPress(L2)) {
      
      int stickY = myPS3->getAnalogHat(LeftHatY);

      headValue = map(stickY, 255, 0, 400, 200);
      pwm.setPWM(2, 0, headValue);
      
    } else if (myPS3->getButtonPress(L1)) {
      
      int stickY = myPS3->getAnalogHat(LeftHatY);

      headValue = map(stickY, 255, 0, NOD_BAR_MAX, NOD_BAR_MIN);
      pwm.setPWM(3, 0, headValue);
    }
  }
  return false;
}


void ps3ToggleSettings(PS3BT* myPS3 = PS3Nav)
{
  if (myPS3->getButtonPress(PS) && myPS3->getButtonClick(L3))
  {
    //Quick Shutdown of PS3 Controller
    output += "\r\nDisconnecting the controller.\r\n";
    myPS3->disconnect();
  }
  if(myPS3->getButtonPress(L1)&&myPS3->getButtonClick(CIRCLE))
  {
    HeadState = 1;
    Serial.print ("HeadState = 1");
  }

   if(myPS3->getButtonPress(L1)&&myPS3->getButtonClick(CROSS))
  {
    HeadState = 0;
    Serial.print ("HeadState = 0");
  }

  //// enable / disable right stick & play sound
  if (myPS3->getButtonPress(L2) && myPS3->getButtonClick(CROSS))
  {
#ifdef SHADOW_DEBUG
    output += "Disiabling the DriveStick\r\n";
#endif
    isStickEnabled = false;
    //        MP3.play(52);
  }
  if (myPS3->getButtonPress(L2) && myPS3->getButtonClick(CIRCLE))
  {
#ifdef SHADOW_DEBUG
    output += "Enabling the DriveStick\r\n";
#endif
    isStickEnabled = true;
    ///        MP3.play(53);
  }
}

void processSoundCommand(char soundCommand)
{
  switch (soundCommand)
  {
    case '-':
#ifdef SHADOW_DEBUG
      output += "Volume Down\r\n";
#endif
      if (vol > 0)
      {

#ifdef Sparkfun
        vol = vol-5;
        if (vol < 0 ) { vol = 0 ;}
        MP3.setVolume(vol);
#endif

      }
      break;
    case '+':
#ifdef SHADOW_DEBUG
      output += "Volume Up\r\n";
#endif


#ifdef Sparkfun
        // The MP3Trigger use a 0-255 vol range
        if (vol < 255) {
          vol = vol+5;
          if (vol > 255 ) { vol = 255 };
          MP3.setVolume(vol);
        }
#endif

        break;

      case '1':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Scream\r\n";
#endif
        musicPlayer.playFullFile("track001.mp3");
        
        break;
        
      case '2':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Doo Doo.\r\n";
#endif
        musicPlayer.playFullFile("track002.mp3");
        break;
        
      case '3':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Scramble\r\n";
#endif
        musicPlayer.playFullFile("track003.mp3");
        break;
        
        case '4':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Scramble\r\n";
#endif
        musicPlayer.playFullFile("track004.mp3");
        break;
        
      case '5':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Mouse Sound.\r\n";
#endif
        musicPlayer.playFullFile("track005.mp3");
        break;
        
      case '6':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Crank Sound.\r\n";
#endif
        musicPlayer.playFullFile("track006.mp3");
        break;
        
      case '7':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Splat.\r\n";
#endif
        musicPlayer.playFullFile("track007.mp3");
        break;
        
      case '8':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Electrical.\r\n";
#endif

        break;
        
      case '9':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play March L1+L.\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(18);
#endif

        break;
      case '0':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Cantina L1+R\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(19);
#endif

        break;
      case 'A':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Jabba Flow L1+O\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(20);
#endif

        break;
      case 'B':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play SWG Music L1+X\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(21);
#endif

        break;
      case 'C':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play MSE Loop L1+L3\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(22);
#endif

        break;
      case 'D':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Meco StarWars L1+PS\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(23);
#endif

        break;
      case 'E':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Meco StarWars L1+PS\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(23);
#endif

        break;
      case 'F':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Meco StarWars L1+PS\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(23);
#endif

        break;
      case 'R':
#ifdef SHADOW_DEBUG
        output += "Sound Button ";
        output += soundCommand;
        output += " - Play Random Mouse Sound. L3\r\n";
#endif

#ifdef Sparkfun
        MP3.trigger(random(1, 15));
#endif

        break;
      default:
#ifdef SHADOW_DEBUG
        output += "Invalid Sound Command\r\n";
#endif

        output += "Invalid Sound Command\r\n";

#ifdef Sparkfun
        MP3.trigger(60);
#endif
      }
  }

  void ps3soundControl(PS3BT* myPS3 = PS3Nav, int controllerNumber = 1)
  {
    if (!(myPS3->getButtonPress(L1) || myPS3->getButtonPress(L2) || myPS3->getButtonPress(PS))) {
      if (myPS3->getButtonClick(UP))          processSoundCommand('1');
      else if (myPS3->getButtonClick(RIGHT))  processSoundCommand('2');
      else if (myPS3->getButtonClick(DOWN))   processSoundCommand('3');
      else if (myPS3->getButtonClick(LEFT))   processSoundCommand('4');
      else if (myPS3->getButtonClick(CROSS))  processSoundCommand('5');
      else if (myPS3->getButtonClick(CIRCLE)) processSoundCommand('6');
      else if (myPS3->getButtonClick(L3))     processSoundCommand('R');
    }
    else if (myPS3->getButtonPress(L2)) {
      if (myPS3->getButtonClick(UP))          processSoundCommand('7');
      else if (myPS3->getButtonClick(RIGHT))  processSoundCommand('8');
      else if (myPS3->getButtonClick(DOWN))   processSoundCommand('9');
      else if (myPS3->getButtonClick(LEFT))   processSoundCommand('0');
      else if (myPS3->getButtonClick(CIRCLE)) processSoundCommand('A');
      else if (myPS3->getButtonClick(CROSS))  processSoundCommand('B');
      else if (myPS3->getButtonClick(L3))     processSoundCommand('R');
    }
    else if (myPS3->getButtonPress(L1)) {
      if (myPS3->getButtonClick(UP))          processSoundCommand('+');
      else if (myPS3->getButtonClick(DOWN))   processSoundCommand('-');
      else if (myPS3->getButtonClick(LEFT))   processSoundCommand('C');
      else if (myPS3->getButtonClick(RIGHT))  processSoundCommand('D');
      else if (myPS3->getButtonClick(CIRCLE)) processSoundCommand('E');
      else if (myPS3->getButtonClick(CROSS))  processSoundCommand('F');
      else if (myPS3->getButtonClick(L3))     processSoundCommand('R');
      else if (myPS3->getButtonClick(PS))     processSoundCommand('D');
    }
  }

  void Drive()
  {
    //Flood control prevention
    if ((millis() - previousMillis) < serialLatency) return;
    if (PS3Nav->PS3NavigationConnected) ps3Drive(PS3Nav);
    //TODO:  Drive control must be mutually exclusive - for safety
    //Future: I'm not ready to test that before FanExpo
    //if (PS3Nav2->PS3NavigationConnected) ps3Drive(PS3Nav2);
  }

  void toggleSettings()
  {
    if (PS3Nav->PS3NavigationConnected) ps3ToggleSettings(PS3Nav);
    if (PS3Nav2->PS3NavigationConnected) ps3ToggleSettings(PS3Nav2);
  }

  void soundControl()
  {
    if (PS3Nav->PS3NavigationConnected) ps3soundControl(PS3Nav, 1);
    if (PS3Nav2->PS3NavigationConnected) ps3soundControl(PS3Nav2, 2);
  }
#ifdef TEST_CONROLLER
  void testPS3Controller(PS3BT* myPS3 = PS3Nav)
  {
    if (myPS3->PS3Connected || myPS3->PS3NavigationConnected) {
      if (myPS3->getButtonPress(PS) && (myPS3->getAnalogHat(LeftHatX) > 137 || myPS3->getAnalogHat(LeftHatX) < 117 || myPS3->getAnalogHat(LeftHatY) > 137 || myPS3->getAnalogHat(LeftHatY) < 117 || myPS3->getAnalogHat(RightHatX) > 137 || myPS3->getAnalogHat(RightHatX) < 117 || myPS3->getAnalogHat(RightHatY) > 137 || myPS3->getAnalogHat(RightHatY) < 117)) {
        output += "LeftHatX: ";
        output += myPS3->getAnalogHat(LeftHatX);
        output += "\tLeftHatY: ";
        output += myPS3->getAnalogHat(LeftHatY);
        if (myPS3->PS3Connected) { // The Navigation controller only have one joystick
          output += "\tRightHatX: ";
          output += myPS3->getAnalogHat(RightHatX);
          output += "\tRightHatY: ";
          output += myPS3->getAnalogHat(RightHatY);
        }
      }
      //Analog button values can be read from almost all buttons
      if (myPS3->getButtonPress(PS) && (myPS3->getAnalogButton(L2) || myPS3->getAnalogButton(R2)))
      {
        if (output != "")
          output += "\r\n";
        output += "L2: ";
        output += myPS3->getAnalogButton(L2);
        if (myPS3->PS3Connected) {
          output += "\tR2: ";
          output += myPS3->getAnalogButton(R2);
        }
      }
      if (myPS3->getButtonClick(L2)) {
        output += " - L2";
        //myPS3->disconnect();
      }
      if (myPS3->getButtonClick(R2)) {
        output += " - R2";
        //myPS3->disconnect();
      }
      if (output != "") {
        Serial.println(output);
        output = ""; // Reset output string
      }
      if (myPS3->getButtonClick(PS)) {
        output += " - PS";
        //myPS3->disconnect();
      }
      else {
        if (myPS3->getButtonClick(TRIANGLE))
          output += " - Traingle";
        if (myPS3->getButtonClick(CIRCLE))
          output += " - Circle";
        if (myPS3->getButtonClick(CROSS))
          output += " - Cross";
        if (myPS3->getButtonClick(SQUARE))
          output += " - Square";

        if (myPS3->getButtonClick(UP)) {
          output += " - Up";
          if (myPS3->PS3Connected) {
            myPS3->setLedOff();
            myPS3->setLedOn(LED4);
          }
        }
        if (myPS3->getButtonClick(RIGHT)) {
          output += " - Right";
          if (myPS3->PS3Connected) {
            myPS3->setLedOff();
            myPS3->setLedOn(LED1);
          }
        }
        if (myPS3->getButtonClick(DOWN)) {
          output += " - Down";
          if (myPS3->PS3Connected) {
            myPS3->setLedOff();
            myPS3->setLedOn(LED2);
          }
        }
        if (myPS3->getButtonClick(LEFT)) {
          output += " - Left";
          if (myPS3->PS3Connected) {
            myPS3->setLedOff();
            myPS3->setLedOn(LED3);
          }
        }

        if (myPS3->getButtonClick(L1))
          output += " - L1";
        if (myPS3->getButtonClick(L3))
          output += " - L3";
        if (myPS3->getButtonClick(R1))
          output += " - R1";
        if (myPS3->getButtonClick(R3))
          output += " - R3";

        if (myPS3->getButtonClick(SELECT)) {
          output += " - Select";
          myPS3->printStatusString();
        }
        if (myPS3->getButtonClick(START)) {
          output += " - Start";
        }
      }
    }
  }
#endif

/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}
