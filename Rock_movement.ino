/*  This code gets the Gripper and Playstation controller working.
 *  Date: 10/20/22
 *  Author:Abigail Doyle
*/

//All the libraries using in the code
#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>   //Robot pin library
#include <SimpleRSLK.h>  //Robot library
#include "PS2X_lib.h"    //Playstation controller library
#include <Servo.h>       //Gripper library

//PS2 controller bluetooth dongle pins
#define PS2_DAT         14 //P1.7 <-> brown wire
#define PS2_CMD         15 //P1.6 <-> orange wire
#define PS2_SEL         34 //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK         35 //P6.7 <-> blue wire
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x;        //PS2 Controller Class and object
Servo gripper;    //Declares Gripper as the servo object

uint16_t normalSpeed = 10;
uint16_t fastSpeed = 20;

int error = 0;
byte type = 0;
byte vibrate = 0;

//All the cases used in switch(STATE)
#define IDLE 4
//Cases for Gripper
#define OPEN 5
#define CLOSE 6
//Cases for movement
#define FORWARD 7
#define BACKWARD 8
#define SPINLEFT 9
#define SPINRIGHT 10
#define TURNINGRIGHT 11
#define TURNINGLEFT 12

int STATE = IDLE;   //Starts the robot in the IDLE case
#define MS 1000

void setup() {
  Serial.begin(57600); //changed from Arduino deafult of 9600
  gripper.attach(SRV_0);
  gripper.write(5);
  setupRSLK();      //Sets up the DC motor pins (aka the wheel pins)
  delayMicroseconds(500 * 1000); //added delay for ps2 module
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = 1;
  while (error) {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    if (error == 0) {
      Serial.print("Found Controller, configured successful ");
      Serial.print("pressures = ");
      if (pressures)
        Serial.println("true ");
      else
        Serial.println("false");
      Serial.print("rumble = ");
      if (rumble)
        Serial.println("true)");
      else
        Serial.println("false");
    }  else if (error == 1)
      Serial.println("No controller found, check wiring");
    else if (error == 2)
      Serial.println("Controller found but not accepting commands.");
    else if (error == 3)
      Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    delayMicroseconds(1000 * 1000);
  }
  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found and ready to use.");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
  }
}

void loop() {
  detect();
  switch (STATE) {
    case IDLE:
      IdleState();
      break;
    case CLOSE:
      CloseState();
      break;
    case OPEN:
      OpenState();
      break;
    case FORWARD:
      forwardState();
      break;
    case BACKWARD:
      backwardState();
      break;
    case SPINLEFT:
      spinLeft();
      break;
    case SPINRIGHT:
      spinRight();
      break;
    case TURNINGLEFT:
      turningLeft();
      break;
    case TURNINGRIGHT:
      turningRight();
      break;
    default:
      break;
  }
}

//Tells robot to move forward
void forwardState(){
  Serial.println("Moving forward.");
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,normalSpeed);
  delayMicroseconds(10 * MS); //can’t use delay
  STATE = IDLE;   //Switches state back to IDLE
}

//Tells robot to move backward
void backwardState(){
  Serial.println("Moving backward.");
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS,normalSpeed);
  delayMicroseconds(10 * MS); //can’t use delay
  STATE = IDLE;
}

//Tells robot to spin counter-clockwise
void spinLeft(){
  Serial.println("Spinning Left.");
  enableMotor(RIGHT_MOTOR);
  disableMotor(LEFT_MOTOR);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR,fastSpeed);
  delayMicroseconds(10 * MS); //can’t use delay
  STATE = IDLE;
}

//Tells robot to spin clockwise
void spinRight(){
  Serial.println("Spinning Right.");
  enableMotor(LEFT_MOTOR);
  disableMotor(RIGHT_MOTOR);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR,fastSpeed);
  delayMicroseconds(10 * MS);
  STATE = IDLE;
}

//Tells robot to turn counter-clockwise
void turningLeft(){
  Serial.println("Turning Left");
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(LEFT_MOTOR,normalSpeed);
  setMotorSpeed(RIGHT_MOTOR,fastSpeed);
  delayMicroseconds(10 * MS);
  STATE = IDLE;
}

//Tells robot to turn clockwise
void turningRight(){
  Serial.println("Turning Right");
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(RIGHT_MOTOR,normalSpeed);
  setMotorSpeed(LEFT_MOTOR,fastSpeed);
  delayMicroseconds(10 * MS);
  STATE = IDLE;
}

void CloseState() {
  gripper.write(30);
  Serial.println("Close Claw");
  delayMicroseconds(10 * MS);
  STATE = IDLE;
}

void OpenState() {
  gripper.write(160);
  Serial.println("Open Claw");
  delayMicroseconds(10 * MS);
  STATE = IDLE;
}

//Reads what button is pressed then goes into the respective state.
void IdleState() {
  if (ps2x.ButtonPressed(PSB_SQUARE))  {        //If Square is pressed
    STATE = OPEN;
  } else if(ps2x.ButtonPressed(PSB_TRIANGLE)){  //If Triangle is pressed
    STATE = CLOSE;
  } else if(ps2x.Button(PSB_PAD_UP) && ps2x.Button(PSB_PAD_LEFT)){    //If up and left is pressed
    STATE = TURNINGLEFT;
  } else if(ps2x.Button(PSB_PAD_UP) && ps2x.Button(PSB_PAD_RIGHT)){   //If up and right is pressed
    STATE = TURNINGRIGHT;
  } else if(ps2x.Button(PSB_PAD_UP)){       //If up is pressed
    STATE = FORWARD;
  } else if(ps2x.Button(PSB_PAD_DOWN)){     //If down is pressed
    STATE = BACKWARD;
  } else if(ps2x.Button(PSB_PAD_LEFT)){     //If left is pressed
    STATE = SPINLEFT;
  } else if(ps2x.Button(PSB_PAD_RIGHT)){    //If right is pressed
    STATE = SPINRIGHT;
  } else {
    STATE = IDLE;     //Restarts the IdleState function until a button is pressed/read by robot
    disableMotor(BOTH_MOTORS);    //Causes the wheels to stop spinning IF you aren't holding the button down
  }
}

void detect() {
  if (error == 1) { //skip loop if no controller found
    return;
  }
  if (type != 2) { //DualShock or Wireless DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
  }
  delayMicroseconds(50 * 1000);
}
