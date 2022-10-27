/*  This code gets the Gripper and Playstation controller working w/Joysticks.
 *  Joysticks working, but mapping needs to be fixed up.
 *  Date: 10/27/22
 *  Author:Abigail Doyle
*/

//All the libraries used for the robot for work
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
#define PS2_DAT         14 //P1.7 = brown wire
#define PS2_CMD         15 //P1.6 = orange wire
#define PS2_SEL         34 //P2.3 = yellow wire (attention)
#define PS2_CLK         35 //P6.7 = blue wire
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

#define pressures   false   //Not using pressure sensors on controller
#define rumble      false   //Not using rumble on controller

PS2X ps2x;        //PS2 Controller Class and object
Servo gripper;    //Declares Gripper as the servo object

uint16_t normalSpeed = 20;  //Variable that sets normal speed to 20
uint16_t fastSpeed = 40;    //Variable that sets normal speed to 40

int error = 0;    //Variable for error
byte type = 0;    
byte vibrate = 0;

//All the cases used in switch(STATE)
#define IDLE 1
//Cases for Gripper
#define OPEN 2
#define CLOSE 3

int STATE = IDLE;   //Starts the robot in the IDLE case
#define MS 1000     //Variable for changing the milliseconds to seconds

int xVal, yVal;     //Variables for x-values and y-values

void setup() {
  Serial.begin(57600); Serial1.begin(57600);  //Sets the baud rate to read and print to the serial monitor.
  gripper.attach(SRV_0);    //Initializes the gripper servo
  gripper.write(5);         //Sets the grippers position to 5 degrees when program is first run
  setupRSLK();      //Sets up the DC motor pins (aka the wheel pins)
  delayMicroseconds(500 * MS); //added delay for ps2 module
  error = 1;    //check for error
  while (error) {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);   //Sets up pins and settings for the PS Controller 
    delayMicroseconds(1000 * MS);   //Delay for a full second
  }
  type = ps2x.readType();   //Reads Controller type
//Reads which controller is being used and prints it to the serial monitor
}

void loop() {
  schmoove();       //Always runs the movement code
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

//This function constantly reads the joystick values and makes the robot move
void schmoove(){
  if (error == 1) { //skip loop if no controller found
    return;
  }
  if (type != 2) { //DualShock or Wireless DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
  }
  delayMicroseconds(50 * MS);
//Y-Values
  yVal = ps2x.Analog(PSS_LY);           //Assigns the variable yVal, to the left Y joystick values
  yVal = map(yVal, 0, 1024, 0, 100);    //Changes the yVal from 0-1024 to 0-40
//X-Values
  xVal = ps2x.Analog(PSS_RX);           //Assigns the variable xVal, to the right X joystick values
  xVal = map(xVal, 0, 1024, 0, 100);    //Changes the xVal from 0-1024 to 0-40
  Serial.print("Y-Values: ");Serial.print(yVal);        Serial1.print("Y-Values: ");Serial1.print(yVal);
  Serial.print(" || X-Values: ");Serial.println(xVal);  Serial1.print(" || X-Values: ");Serial1.println(xVal);
  enableMotor(BOTH_MOTORS);   //Turns on both motors
  if (yVal > 25){           //If the yVal is more than 25, go forward
    setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);   //Sets both motors to forward
    setMotorSpeed(BOTH_MOTORS,yVal);                    //Sets motor speed to yVal.
    Serial.println("Going Forward"); Serial1.println("Going Forward");
  } else if(yVal < 25){     //If the yVal is less than 25, go backward
    setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);  //Sets both motors to backward
    setMotorSpeed(BOTH_MOTORS,yVal);                    //Sets motor speed to yVal
    Serial.println("Backing Up"); Serial1.println("Backing Up");
  } else if (xVal > 25){    //If the xVal is more than 25, turn right
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);    //Sets the left motor to forward
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);  //Sets the right motor to backward
    setMotorSpeed(BOTH_MOTORS,xVal);                    //Sets both motors speed to xVal
    Serial.println("Turning Right"); Serial1.println("Turning Right");
  } else if(xVal < 25){     //If the xVal is less than 25, turn left
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);   //Sets the left motor to backward
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);   //Sets the right motor to forward
    setMotorSpeed(BOTH_MOTORS,xVal);                    //Sets both motors speed to xVal
    Serial.println("Turning Left"); Serial1.println("Turning Left");
  } else {
    disableMotor(BOTH_MOTORS);    //Disables both of the motors if both xVal && yVal = 25
  }
}

//Closes the gripper
void CloseState() {
  gripper.write(30);              //Closes gripper to 30 degrees
  Serial.println("Close Claw");   //Prints indicator to serial monitor
  delayMicroseconds(10 * MS);
  STATE = IDLE;                   //Returns the state to IDLE
}

//Opens the gripper
void OpenState() {
  gripper.write(160);           //Opens gripper to 160 degrees
  Serial.println("Open Claw");  //Prints indicator to serial monitor
  delayMicroseconds(10 * MS);
  STATE = IDLE;                 //Returns the state to IDLE
}

//Reads what button is pressed then goes into the respective state.
void IdleState() {
  if (ps2x.ButtonPressed(PSB_SQUARE))  {        //If square button is pressed...
    STATE = OPEN;     //Changes state to OPEN
  } else if(ps2x.ButtonPressed(PSB_TRIANGLE)){  //If triangle button is pressed... 
    STATE = CLOSE;    //Changes state to CLOSE
  } else {
    STATE = IDLE;     //Restarts the IdleState function until a button is pressed/read by robot
  }
}
