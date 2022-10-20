#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>
#include <Servo.h>
#include <TinyIR.h>

#define INCLUDE_REPEATS

#define MS 1000

int IRpin = 33;

IRData IRresults;
Servo gripper;

void setup() {
  Serial.begin(57600);
  delayMicroseconds(100*MS);
  initTinyIRReceiver();
  gripper.attach(SRV_0);
  pinMode(IRpin, INPUT);
  setupRSLK();
}

void loop() {
uint16_t normalSpeed = 10;
  if(decodeIR(&IRresults)){
    Serial.print(IRresults.command, HEX);
    translateIR();
  }
}

void translateIR(){ 
  uint16_t normalSpeed = 10;
  uint16_t fastSpeed = 20;
  Serial.print("translate IR: ");
  switch(IRresults.command){
    case 0x45:
      Serial.println("POWER");
      enableMotor(RIGHT_MOTOR);
      disableMotor(LEFT_MOTOR);
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(RIGHT_MOTOR,normalSpeed);
      break;
    case 0x46:
      Serial.println("VOL+");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
       setMotorSpeed(LEFT_MOTOR,normalSpeed);
       setMotorSpeed(RIGHT_MOTOR,normalSpeed);
      break;
    case 0x47:
      Serial.println("FUNC");
      enableMotor(LEFT_MOTOR);
      disableMotor(RIGHT_MOTOR);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR,normalSpeed);
      break;
    case 0x44:
      Serial.println("LEFT");
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(LEFT_MOTOR,normalSpeed);
      setMotorSpeed(RIGHT_MOTOR,fastSpeed);
      break;
    case 0x40:
      Serial.println("PLAY");
      disableMotor(BOTH_MOTORS);
      break;
    case 0x43:
      Serial.println("RIGHT");
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(LEFT_MOTOR,fastSpeed);
      setMotorSpeed(RIGHT_MOTOR,normalSpeed);
      break;
    case 0x15:
       Serial.println("VOL-");
       enableMotor(BOTH_MOTORS);
       setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);
       setMotorSpeed(LEFT_MOTOR,normalSpeed);
       setMotorSpeed(RIGHT_MOTOR,normalSpeed);
      break;
    case 0x19:
      Serial.println("EQ");
      gripper.write(150);
      break;
    case 0xD:
      Serial.println("ST");
      gripper.write(40);
      break;
    default:
      break;
  }
  delayMicroseconds(100*MS);
}
