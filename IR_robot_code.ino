//All of the Robot Libraries
#include <Bump_Switch.h>        //Library for Bumpers
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>          //Robot Pins
#include <SimpleRSLK.h>         //Robot Library
#include <Servo.h>              //Gripper Library
#include <TinyIR.h>             //IR Sensor Library

#define MS 1000

int IRpin = 33;
int sharppin = 23;
int stopDistance = 5;

uint16_t normalSpeed = 20;
uint16_t fastSpeed = 40;

IRData IRresults;
Servo gripper;

void setup() {
  Serial.begin(57600); Serial1.begin(57600);
  delayMicroseconds(100 * MS);
  initTinyIRReceiver();
  gripper.attach(SRV_0);
  pinMode(IRpin, INPUT);
  pinMode(sharppin, INPUT);
  setupRSLK();                  //Setups Robot
}

void loop() {
  decodeIR(&IRresults);
  if (IRresults.command == 28) {
    autonomous();
  } else {
    buttonPress();
  }
}

void autonomous() {
  uint16_t normalSpeed = 20;
  uint16_t fastSpeed = 40;
  int distance = analogRead(sharppin);
  Serial.println(distance);
  if (distance < 300) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
  }
  else if (distance < 600) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
  }
  else {
    disableMotor(BOTH_MOTORS);
    gripper.write(150);
  }
}

void AUTO(){
        decodeIR(&IRresults);
      int button = IRresults.command;
      Serial.println(button);
      if(button == 28){
       int STATE = 0x1C;
      }
      else{
       int STATE = IRresults.command;
      }
}

void buttonPress() {
  uint16_t normalSpeed = 20;
  uint16_t fastSpeed = 40;
  Serial.print("translate IR: ");
  int STATE = IRresults.command;
  switch (STATE) {
    case 0x45:
      Serial.println("POWER");
      enableMotor(RIGHT_MOTOR);
      disableMotor(LEFT_MOTOR);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      break;
    case 0x46:
      Serial.println("VOL+");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      break;
    case 0x47:
      Serial.println("FUNC");
      enableMotor(LEFT_MOTOR);
      disableMotor(RIGHT_MOTOR);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      break;
    case 0x44:
      Serial.println("LEFT");
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      setMotorSpeed(RIGHT_MOTOR, fastSpeed);
      break;
    case 0x40:
      Serial.println("PLAY");
      disableMotor(BOTH_MOTORS);
      break;
    case 0x43:
      Serial.println("RIGHT");
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(LEFT_MOTOR, fastSpeed);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      break;
    case 0x15:
      Serial.println("VOL-");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      break;
    case 0x19:
      Serial.println("EQ");
      gripper.write(150);
      break;
    case 0xD:
      Serial.println("ST");
      gripper.write(40);
      break;
    case 0x1C:
      Serial.println("5");
      Serial.println("Now in Autonomous Mode");
      autonomous();
      break;
    default:
      break;
  }
  delayMicroseconds(100 * MS);
}
