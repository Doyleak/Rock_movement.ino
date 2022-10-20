#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>
#include <Servo.h>
#include <TinyIR.h>

#define MS 1000

int IRpin = 33;

uint16_t normalSpeed = 20;
uint16_t fastSpeed = 40;

IRData IRresults;
Servo gripper;

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);
  delayMicroseconds(100*MS);
  initTinyIRReceiver();
  gripper.attach(SRV_0);
  gripper.write(30);
  pinMode(IRpin, INPUT);
  setupRSLK();
}

void loop() {
  if(decodeIR(&IRresults)){
    Serial.print(IRresults.command, HEX);
    Serial1.print(IRresults.command, HEX);
    translateIR();
  }
}

void translateIR(){ 
  switch(IRresults.command){
    case 0x45:
      Serial.println("Spin Left");
      Serial1.println("Spin Left");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(RIGHT_MOTOR,fastSpeed);
      setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR,fastSpeed);
      break;
    case 0x46:
      Serial.println("Move Forward");
      Serial1.println("Move Forward");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
       setMotorSpeed(BOTH_MOTORS,fastSpeed);
      break;
    case 0x47:
      Serial.println("Spin Right");
      Serial1.println("Spin Right");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR,fastSpeed);
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
      setMotorSpeed(RIGHT_MOTOR,fastSpeed);
      break;
    case 0x44:
      Serial.println("Turn Left");
      Serial1.println("Turn Left");
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(LEFT_MOTOR,normalSpeed);
      setMotorSpeed(RIGHT_MOTOR,fastSpeed);
      break;
    case 0x40:
      Serial.println("Stop Moving");
      Serial1.println("Stop Moving");
      disableMotor(BOTH_MOTORS);
      break;
    case 0x43:
      Serial.println("Turn Right");
      Serial1.println("Turn Right");
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(LEFT_MOTOR,fastSpeed);
      setMotorSpeed(RIGHT_MOTOR,normalSpeed);
      break;
    case 0x15:
      Serial.println("Moving Backwards");
      Serial1.println("Moving Backwards");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);
      setMotorSpeed(BOTH_MOTORS,fastSpeed);
      break;
    case 0x19:
      Serial.println("Open Claw");
      Serial1.println("Open Claw");
      gripper.write(160);
      break;
    case 0xD:
      Serial.println("Close Claw");
      Serial1.println("Close Claw");
      gripper.write(30);
      break;
    default:
      break;
  }
  delayMicroseconds(100*MS);
}
