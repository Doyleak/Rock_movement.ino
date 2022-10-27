/*  This code gets the Gripper and Playstation controller working.
 *  Work in Progress for Joystick working.
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

uint16_t normalSpeed = 20;
uint16_t fastSpeed = 40;

int error = 0;
byte type = 0;
byte vibrate = 0;

//All the cases used in switch(STATE)
#define IDLE 4
//Cases for Gripper
#define OPEN 5
#define CLOSE 6

int STATE = IDLE;   //Starts the robot in the IDLE case
#define MS 1000

int xVal, yVal;

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
    delayMicroseconds(1000 * 1000);
  }
  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      Serial1.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found and ready to use.");
      Serial1.print("DualShock Controller found and ready to use.");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      Serial1.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      Serial1.print("Wireless Sony DualShock Controller found ");
      break;
  }
}

void loop() {
  schmoove();
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
    default:
      break;
  }
}

void schmoove(){
  if (error == 1) { //skip loop if no controller found
    return;
  }
  if (type != 2) { //DualShock or Wireless DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
  }
  delayMicroseconds(50 * 1000);
//Y-Values
  yVal = ps2x.Analog(PSS_LY);           //Assigns the variable yVal, to the left Y joystick values
  yVal = map(yVal, 0, 1024, 0, 100);     //Changes the yVal from 0-1024 to 0-40
//X-Values
  xVal = ps2x.Analog(PSS_RX);           //Assigns the variable xVal, to the right X joystick values
  xVal = map(xVal, 0, 1024, 0, 100);     //Changes the xVal from 0-1024 to 0-40
  Serial.print("Y-Values: ");Serial.print(yVal);Serial1.print("Y-Values: ");Serial1.print(yVal);
  Serial.print(" || X-Values: ");Serial.println(xVal);Serial1.print(" || X-Values: ");Serial1.println(xVal);
  enableMotor(BOTH_MOTORS);   //Turns on both motors
  if (yVal > 25){
    setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); //Sets both motors to forward
    setMotorSpeed(BOTH_MOTORS,yVal);                  //Sets motor speed to yVal.
    Serial.println();
  } else if(yVal < 25){
    setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);
    setMotorSpeed(BOTH_MOTORS,yVal);
  } else if (xVal > 25){
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorSpeed(BOTH_MOTORS,xVal);
  } else if(xVal < 25){
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS,xVal);
  } else {
    disableMotor(BOTH_MOTORS);
  }
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
  if (ps2x.ButtonPressed(PSB_SQUARE))  {        //If Square is pressed, changes state to open
    STATE = OPEN;
  } else if(ps2x.ButtonPressed(PSB_TRIANGLE)){  //If Triangle is pressed, changes state to close.
    STATE = CLOSE;
  } else {
    STATE = IDLE;   //Restarts the IdleState function until a button is pressed/read by robot
  }
}
