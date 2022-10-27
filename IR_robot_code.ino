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
int sharppin = 23;
int stopDistance = 5;

IRData IRresults;
Servo gripper;

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint8_t lineMode = 0;
uint8_t lineColor;
uint32_t linePos;

uint16_t normalSpeed = 20;
uint16_t turnSpeed = 40;
uint16_t fastSpeed = 28;

int rightsensval;
int linePosition = 0;
int photores = A7;
int led = 7;
int light = 0;

bool isCalibrationComplete = false;

void setup() {
  Serial.begin(57600);  Serial1.begin(57600);
  delayMicroseconds(100 * MS);
  initTinyIRReceiver();
  gripper.attach(SRV_0);
  Serial.println("test");
  pinMode(IRpin, INPUT);
  pinMode(sharppin, INPUT);
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);
  pinMode(led, OUTPUT);
  pinMode(photores, INPUT);
  setupRSLK();
}

void loop() {
  Light();
  if (isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }
  decodeIR(&IRresults);
  if(IRresults.command == 82){
    autonomousLine();
  }
   else if (IRresults.command == 28) {
    autonomousDist();
  }
  else {
    translateIR();
  }
}

void autonomousLine(){
  lineColor = LIGHT_LINE;
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
  linePos = getLinePosition(sensorCalVal,lineColor);
  Serial.println(sensorCalVal[6]);    Serial1.println(sensorCalVal[6]);
  Serial.println(sensorCalVal[7]);    Serial1.println(sensorCalVal[7]);
  int rightsensval = sensorCalVal[6];  
  if(linePos > 0 && linePos < 3000) {
    setMotorSpeed(LEFT_MOTOR,normalSpeed);
    setMotorSpeed(RIGHT_MOTOR,fastSpeed);
  } else if(linePos > 3500) {
    setMotorSpeed(LEFT_MOTOR,fastSpeed);
    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
  } else if(rightsensval > 500){
    disableMotor(BOTH_MOTORS);
    setMotorSpeed(LEFT_MOTOR, turnSpeed);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(RIGHT_MOTOR, turnSpeed);
    enableMotor(BOTH_MOTORS);
    delayMicroseconds(500000);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
  } else {
    setMotorSpeed(LEFT_MOTOR,normalSpeed);
    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
  }
}

void autonomousDist() {
  uint16_t normalSpeed = 18;
  uint16_t fastSpeed = 30;
  int distance = analogRead(sharppin);
  Serial.println(distance);   Serial1.println(distance);
  if (distance >= 600) {
    disableMotor(BOTH_MOTORS);
    delayMicroseconds(1000 * MS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
    delayMicroseconds(500 * MS);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
    delayMicroseconds(1000 * MS);
    disableMotor(BOTH_MOTORS);
    gripper.write(150);
  } else if (distance < 600){
    enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
  } else {
    setMotorSpeed(LEFT_MOTOR,normalSpeed);
    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
  }
}

void translateIR() {
  Serial.print("translate IR: ");   Serial1.print("translate IR: ");
  int STATE = IRresults.command;
  switch (STATE) {
    case 0x45:
      Serial.println("POWER");    Serial1.println("POWER");
      enableMotor(RIGHT_MOTOR);
      disableMotor(LEFT_MOTOR);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      break;
    case 0x46:
      Serial.println("VOL+");   Serial1.println("VOL+");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      break;
    case 0x47:
      Serial.println("FUNC");   Serial1.println("FUNC");
      enableMotor(LEFT_MOTOR);
      disableMotor(RIGHT_MOTOR);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      break;
    case 0x44:
      Serial.println("LEFT");   Serial1.println("LEFT");
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      setMotorSpeed(RIGHT_MOTOR, fastSpeed);
      break;
    case 0x40:
      Serial.println("PLAY");   Serial1.println("PLAY");
      disableMotor(BOTH_MOTORS);
      break;
    case 0x43:
      Serial.println("RIGHT");    Serial1.println("RIGHT");
      enableMotor(BOTH_MOTORS);
      setMotorSpeed(LEFT_MOTOR, fastSpeed);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      break;
    case 0x9:
      Serial.println("UP");   Serial1.println("UP");
      break;
    case 0x15:
      Serial.println("VOL-");   Serial1.println("VOL-");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      break;
    case 0x19:
      Serial.println("EQ");   Serial1.println("EQ");
      gripper.write(150);
      break;
    case 0xD:
      Serial.println("ST");   Serial1.println("ST");
      gripper.write(40);
      break;
    case 0x1C:
      Serial.println("5");  Serial1.println("5");
      Serial.println("Now in Autonomous Mode");   Serial1.println("Now in Autonomous Mode");
      break;
    default:
      break;
  }
  delayMicroseconds(100 * MS);
}

void floorCalibration() {
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  delay(1000);
  Serial.println("Running calibration on floor");   Serial1.println("Running calibration on floor");
  simpleCalibrate();      //Runs the calibration function
  Serial.println("Reading floor values complete");    Serial1.println("Reading floor values complete");
  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  delay(1000);
  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD); //Set both motors direction forward
  enableMotor(BOTH_MOTORS);         //Enable both motors
  setMotorSpeed(BOTH_MOTORS, 20);   //Sets both motors speed to 20
  for (int x = 0; x < 100; x++) {
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
  }
  disableMotor(BOTH_MOTORS);  //Disables both motors
}

void Light(){
  light = analogRead(photores);
  if (light > 0) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }
}
