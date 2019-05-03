//=============================1================================================
// Based upon Kurt's PX Reactor arm code.
// https://github.com/KurtE
// This code provides serial control of the Interbotix line of robotic arms, which are sold by Trossen
// Robotics: http://www.trossenrobotics.com/robotic-arms.aspx
// http://learn.trossenrobotics.com/interbotix/robot-arms
//=============================================================================
//armlink packet structure is as follows
//
// 255, XH, XL, YH, YL, ZH, ZL, WAH, WAL, WRH, WAL, GH, GL, DTIME, BUTTONS, EXT, CHECKSUM
//
// (please note, actual XYZ min/max for specific arm defined below)
//
// Protocol value ranges
//
// XH = high byte X-axis
// XL = low byte, 0-1023 (-512 through +512 via armlink)
//
// YH = high byte Y-axis
// YL = low byte, 0-1023
//
// ZH = high byte Z-axis
// ZL = low byte, 0-1023 
//
// WAH = high byte (unused for now, placeholder for higher res wrist angle)
// WAL = low byte, 0-180 (-90 through +90 via armlink)
//
// WRH = high byte 
// WRL = low byte, 0-1023. 512 center
//
// GH = high byte
// GL = low byte, 0-512. 256 center
//
// DTIME = byte. DTIME*16 = interpolation delta time
//
// Buttons = byte (not implemented)
//
// EXT = byte. Extended instruction set.
// EXT < 16 = no action
// EXT = 32 = 3D Cartesian IK
// EXT = 48 = Cylindrical IK Xaxis = 0-4096 value, untested
// EXT = 64 = BackHoe aka passthrough UNTESTED
//
// CHECKSUM = (unsigned char)(255 - (XH+XL+YH+YL+ZH+ZL+WAH+WAL+WRH+WRL+GH+GL+DTIME+BUTTONS+EXT)%256)

//  This code is a Work In Progress and is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
//  
//=============================================================================

//=============================================================================
// Define Options
//=============================================================================

#define PINCHER 1
#define REACTOR 2
#define WIDOWX 3

//#define ARMTYPE PINCHER
#define ARMTYPE REACTOR
//#define ARMTYPE WIDOWX

#if !defined(ARMTYPE) 
#error YOU HAVE TO SELECT THE ARM YOU ARE USING! Uncomment the correct line above for your arm
#endif




#define SOUND_PIN    7      // Tell system we have added speaker to IO pin 1
#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial

//=============================================================================
// Global Include files
//=============================================================================
#include <ax12.h>
#include <BioloidController.h>
#include <ArmLink.h>
#include "InputControl.h"

//=============================================================================
// Global Objects
//=============================================================================
BioloidController bioloid = BioloidController(1000000);
ArmLink armlink = ArmLink();



// Message information
unsigned long   ulLastMsgTime;          // Keep track of when the last message arrived to see if controller off
byte            buttonsPrev;            // will use when we wish to only process a button press once
boolean RunCheck;


//===================================================================================================
// Setup 
//====================================================================================================
void setup() {
  //Serial activity LED output
  pinMode(0,OUTPUT);  
  RunCheck = 0;
  // Lets initialize the Serial Port
  Serial.begin(9600);//38400


    delay(50);
  //IDPacket();
  //Serial.println("Interbotix Robot Arm Online.");


  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
  delay(100);
  // Start off to put arm to sleep...
  //PutArmToSleep();
  //Send ID Packet

  MSound(3, 60, 2000, 80, 2250, 100, 2500);

  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);

  g_bIKStatus = doArmIK(true, 0, 0, 250, 0);//Home claw open
  MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 512, 1000, true); //wristrot formula = 512+(90/0.288)
  delay(1000);


  RunCheck = 1;
  //for(int i = 1; i < 9;i++){ 
  //  ax12SetRegister2(i, AX_GOAL_SPEED_L, 500);
  //}

}


//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() {

  //Serial.println("###########################");
  //Serial.println("Initializing Move Position Test");  
  //Serial.println("###########################");

  delay(100);    // recommended pause

  Serial.println("waiting for object");  
  delay(500);
  if(Serial.available() > 0)
  {
    String msg = "";
    while(Serial.available() > 0)
    {
      msg += char(Serial.read());
      delay(10);
      Serial.flush(); 
    }
    int stringHead(0);
    String storeString[5];
    int pieceOfString(0);
    for (int i(0); i < msg.length(); i++ )
    {
      if (msg[i] == ',')
      {
        storeString[pieceOfString]=msg.substring(stringHead,i);
        stringHead = i+1;
        pieceOfString++;
      }    
    }

    int xy[4];
    for (int i(0); i < 4; i++)
    {
      xy[i] = storeString[i].toInt();
    }

    Serial.print("X: ");
    Serial.println(xy[0]);
    Serial.print("Y: ");
    Serial.println(xy[1]);
    Serial.print("Object: ");
    Serial.println(xy[2]);
    Serial.print("Angle: ");
    Serial.println(xy[3]);

    int x_coord = xy[0];
    int y_coord = xy[1];
    int object = xy[2];
    int angle = xy[3];

    if (angle == 0)
      angle = 0.1;

    if ((object == 1 || object == 2 || object == 3 || object == 4 || object == 5))
    {
      g_bIKStatus = doArmIK(true, 0, 0, 250, 0);//Home claw open
      MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 512, 1000, true);
      delay(500);
      g_bIKStatus = doArmIK(true, x_coord, y_coord, (0.28*y_coord + 7.12), -90);//z = 0.30*y + 7.12 -> compensate for robot's weight
      MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512+(angle/0.288), 512, 2000, true); // 512+(angle/0.288)
      delay(500);
      g_bIKStatus = doArmIK(true, x_coord, y_coord, (0.28*y_coord + 7.2), -90);//Close claw on object
      MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512+(angle/0.288), 20, 1000, true);
      delay(500);
      g_bIKStatus = doArmIK(true, 0, 150, 300, 0);//Home claw closed
      MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 20, 1000, true);
      delay(500);
      if (object == 1)//Yellow bolt
      {
        g_bIKStatus = doArmIK(true, 170, 30, 255, 0);//Yellow Bolts Box (Nuts X:-180 Y:20)
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 20, 1000, true);
        delay(500);
        g_bIKStatus = doArmIK(true, 170, 30, 255, -30);//Yellow Bolts Box claw open
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 512, 1000, true);
        delay(500);
      }
      if (object == 2)//Yellow nut
      {
        g_bIKStatus = doArmIK(true, -180, 0, 250, 0);//Nuts Box
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 20, 1000, true);
        delay(500);
        g_bIKStatus = doArmIK(true, -180, 0, 265, -30);//Nuts Box claw open
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 512, 1000, true);
        delay(500);
      }
      if (object == 3)//Silver Bolt
      {
        g_bIKStatus = doArmIK(true, 180, 130, 250, 0);//Silver Bolts Box
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 20, 1000, true);
        delay(500);
        g_bIKStatus = doArmIK(true, 180, 130, 265, -30);//Silver Bolts Box
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 512, 1000, true);
        delay(500);
      }
      if (object == 4)//Silver Nut
      {
        g_bIKStatus = doArmIK(true, -180, 125, 250, 0);//Silver Nuts Box
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 20, 1000, true);
        delay(500);
        g_bIKStatus = doArmIK(true, -180, 125, 265, -30);//Silver Nuts Box
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 512, 1000, true);
        delay(500);
      }
      if (object == 5)//Other
      {
        g_bIKStatus = doArmIK(true, 240, -90, 330, 0);//Go back to throw object away
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 20, 1000, true);
        delay(500);
        g_bIKStatus = doArmIK(true, 250, -90, 110, -45);//Get down to open claw
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 256, 20, 1000, true); 
        delay(500);
        g_bIKStatus = doArmIK(true, 250, -90, 110, -45);//Open claw
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 256, 512, 1000, true); 
        delay(500);
        g_bIKStatus = doArmIK(true, 240, -90, 330, 0);//Go back up to go back home
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 20, 1000, true);
        delay(500);
      }
      g_fArmActive = false;
      g_bIKStatus = doArmIK(true, 0, 0, 250, 0);//Home claw open
      MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 512, 1500, true);
      delay(500);
    }
    if (object == 6)
    {
      if (x_coord >0)
      {
        g_bIKStatus = doArmIK(true, x_coord+50, y_coord, (0.3*y_coord + 7.12), -90);//z = 0.30*y + 7.12 -> compensate for robot's weight
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512+(angle/0.288), 30, 2000, true); // 512+(angle/0.288)
        delay(500);
        g_bIKStatus = doArmIK(true, x_coord, y_coord, (0.3*y_coord + 7.12), -90);//z = 0.30*y + 7.12 -> compensate for robot's weight
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512+(angle/0.288), 30, 150, true); // 512+(angle/0.288)
        delay(500);
      }
      else
      {
        g_bIKStatus = doArmIK(true, x_coord-50, y_coord, (0.3*y_coord + 7.12), -90);//z = 0.30*y + 7.12 -> compensate for robot's weight
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512+(angle/0.288), 20, 2000, true); // 512+(angle/0.288)
        delay(500);
        g_bIKStatus = doArmIK(true, x_coord, y_coord, (0.3*y_coord + 7.12), -90);//z = 0.30*y + 7.12 -> compensate for robot's weight
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512+(angle/0.288), 20, 150, true); // 512+(angle/0.288)
        delay(500);
      }
    }
    g_fArmActive = false;
    g_bIKStatus = doArmIK(true, 0, 0, 250, 0);//Home claw open
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 512, 1500, true);
    delay(500);
  }
  clearBuffer();
}

void clearBuffer()
{
  //clear out the serial buffer
  byte w = 0;

  for (int i = 0; i < 10; i++)
  {
    while (Serial.available() > 0)
    {
      char k = Serial.read();
      w++;
      delay(1);
    }
    delay(1);
  }
}

// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop

}

void MSound(byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif



