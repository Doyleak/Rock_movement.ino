/*  This code gets the Gripper and Playstation controller working.
 *  Joystick currently does not work >:( Please Fix.
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

PS2X ps2x;        //PS2 Controller Class and object
Servo gripper;    //Declares Gripper as the servo object

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

int STATE = IDLE;   //Starts the robot in the IDLE case
#define MS 1000

int leftYAxisVal;
int rightXAxisVal;

void setup() {
  Serial.begin(57600); //changed from Arduino deafult of 9600
  gripper.attach(SRV_0);
  gripper.write(5);
  setupRSLK();      //Sets up the DC motor pins (aka the wheel pins)
  delayMicroseconds(500 * 1000); //added delay for ps2 module
}

void loop() {
  ps2x.read_gamepad(); //read controller
  leftYAxisVal = ps2x.Analog(PSS_LY);
  leftYAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  rightXAxisVal = ps2x.Analog(PSS_RX);
  rightXAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
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
    default:
      break;
  }
}

//Tells robot to move forward
void forwardState(){
  Serial.println("Moving forward.");
  ps2x.read_gamepad(); //read controller
  leftYAxisVal = ps2x.Analog(PSS_LY);
  leftYAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  rightXAxisVal = ps2x.Analog(PSS_RX);
  rightXAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,leftYAxisVal);
  delayMicroseconds(10 * MS); //can’t use delay
  STATE = IDLE;   //Switches state back to IDLE
}

//Tells robot to move backward
void backwardState(){
  Serial.println("Moving backward.");
  ps2x.read_gamepad(); //read controller
  leftYAxisVal = ps2x.Analog(PSS_LY);
  leftYAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  rightXAxisVal = ps2x.Analog(PSS_RX);
  rightXAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS,leftYAxisVal);
  delayMicroseconds(10 * MS); //can’t use delay
  STATE = IDLE;
}

//Tells robot to spin counter-clockwise
void spinLeft(){
  Serial.println("Spinning Left.");
  ps2x.read_gamepad(); //read controller
  leftYAxisVal = ps2x.Analog(PSS_LY);
  leftYAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  rightXAxisVal = ps2x.Analog(PSS_RX);
  rightXAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS,rightXAxisVal);
  delayMicroseconds(10 * MS); //can’t use delay
  STATE = IDLE;
}

//Tells robot to spin clockwise
void spinRight(){
  Serial.println("Spinning Right.");
  ps2x.read_gamepad(); //read controller
  leftYAxisVal = ps2x.Analog(PSS_LY);
  leftYAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  rightXAxisVal = ps2x.Analog(PSS_RX);
  rightXAxisVal = map(leftYAxisVal, 512, 1024, 0, 50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS,rightXAxisVal);
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
  } else if(leftYAxisVal >= 40){     //If up is pressed
    STATE = FORWARD;
  } else if(leftYAxisVal <= 30){     //If down is pressed
    STATE = BACKWARD;
  } else if(rightXAxisVal >= 40){    //If left is pressed
    STATE = SPINLEFT;
  } else if(rightXAxisVal <= 30){    //If right is pressed
    STATE = SPINRIGHT;
  } else {
    STATE = IDLE;                 //Restarts the IdleState function until a button is pressed/read by robot
    disableMotor(BOTH_MOTORS);    //Causes the wheels to stop spinning IF you aren't holding the button down
  }
}
