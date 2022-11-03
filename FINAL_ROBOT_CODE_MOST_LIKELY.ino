/*  Everything works as planned, except the light sensor is interfering with line followning.
    Date: 10/27/22
    Authors: Abigail Doyle, Catherine Hodge, Noah Zahm, Nicholas Douglas 
*/

//All the libraries used for the robot for work
#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>      //Robot pin library
#include <SimpleRSLK.h>     //Robot library
#include "PS2X_lib.h"       //Playstation controller library
#include <Servo.h>          //Gripper library
#include <TinyIRremote.h>   //IR library

//PS2 controller bluetooth dongle pins
#define PS2_DAT         14 //P1.7 = brown wire
#define PS2_CMD         15 //P1.6 = orange wire
#define PS2_SEL         34 //P2.3 = yellow wire (attention)
#define PS2_CLK         35 //P6.7 = blue wire
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

#define IR_TRX_PIN 17
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define pressures   false   //Not using pressure sensors on controller
#define rumble      false   //Not using rumble on controller

//All the cases used in switch(STATE)
#define IDLE 1
#define AUTO 2
//Cases for Gripper
#define OPEN 3
#define CLOSE 4
#define MOVE 5
#define SONG 6

#define MS 1000     //Variable for changing the milliseconds to seconds

//Speed variables
uint16_t normalSpeed = 25;  //Variable that sets normal speed to 20
uint16_t fastSpeed = 33;    //Variable that sets fast speed to 28
uint16_t turnSpeed = 40;    //Variable that sets turn speed to 40
//Line following variables
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint8_t lineMode = 0;
uint8_t lineColor;
uint32_t linePos;     //Line position variable

int xVal, yVal;       //Variables for x-values and y-values
int STATE = IDLE;     //Starts the robot in the IDLE case
int sharppin = 23;    //
int stopDistance = 5; //
int rightsensval;     //
int leftsensval;
int linePosition = 0; //
int photores = 33;    //
int led = 2;          //LED pin = 7
int light = 0;        //Light value = 0
int Health, Weapon;   //Health and Weapons variables

bool isCalibrationComplete = false;

PS2X ps2x;        //PS2 Controller Class and object
Servo gripper;    //Declares Gripper as the servo object
IRData IRHlP;     //Health Pack
IRData IRWnP;     //Weapons Pack
IRsend sendIR;

// Fortnite Buzzer Constants
int buzzerPin = -1; // buzzer pin is not assigned to anything specific until setup, just set to 0
int noteLength = 0; // noteLength assigned to default value, isn't used
int restLength = 0; // restLength assigned to default value, isn't used
bool testingBuzzer = false; // testingBuzzer assigned to default value, false
float noteSpeed = 0.25; // speed of the notes/rests. 1 being normal tempo, 2 being twice as slow, and 0.5 being twice as fast

void setup() {
  Serial.begin(57600); Serial1.begin(57600);  //Sets the baud rate to read and print to the serial monitor.
  gripper.attach(SRV_0);    //Initializes the gripper servo
  //Sets the grippers position to 5 degrees when program is first run
  pinMode(sharppin, INPUT);
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);
  pinMode(led, OUTPUT);
  pinMode(photores, INPUT);
  setupRSLK();    //Sets up the DC motor pins (aka the wheel pins)
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);   //Sets up pins and settings for the PS Controller
  delayMicroseconds(1000 * MS);   //Delay for a full second
  sendIR.begin(IR_TRX_PIN, true, GREEN_LED);
  setBuzzerPin(37); // default pin for the buzzer
  if (buzzerPin == -1) { // test to make sure buzzer pin is initially assigned
    Serial1.println("Error: Buzzer Pin Assignment Failed");
  }
}

void loop() {
  ps2x.read_gamepad();      //Always runs the schmoove function making the robot move.
  if (isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
    STATE = IDLE;
  }
  switch (STATE) {
    case IDLE:
      idleState();
      break;
    case AUTO:
      autonomousState();
      break;
    case CLOSE:
      closeState();
      break;
    case OPEN:
      openState();
      break;
    case MOVE:
      schmoove();
      IR();
      break;
    case SONG:
      playSong();
      break;
    default:
      break;
  }
}
//This function constantly reads the joystick values and makes the robot move
void schmoove() { //Reads controller
  delayMicroseconds(50 * MS);
  xVal = ps2x.Analog(PSS_RX);           //Assigns the variable xVal, to the right X joystick values
  xVal = map(xVal, 128, 255, 0, 20);    //Changes the xVal from 0-255 to 0-40
  yVal = ps2x.Analog(PSS_LY);           //Assigns the variable yVal, to the left Y joystick values
  yVal = map(yVal, 127, 255, 0, 60);    //Changes the yVal from 0-255 to 0-40
  if (yVal < 0) {          //If the yVal is less than 0, go forward
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);  //Sets both motors to forward
    enableMotor(BOTH_MOTORS);   //Turns on both motors
    setMotorSpeed(BOTH_MOTORS, yVal * -1);                   //Sets motor speed to yVal.
    Serial.println("Going Forward"); Serial1.println("Going Forward");
    Serial.print("Y-Values: ");    Serial.println(yVal);  Serial1.print("Y-Values: ");    Serial1.println(yVal);
  } else if (yVal > 0) {   //If the yVal is less than 0, go backward
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD); //Sets both motors to backward
    enableMotor(BOTH_MOTORS);   //Turns on both motors
    setMotorSpeed(BOTH_MOTORS, yVal);                   //Sets motor speed to yVal
    Serial.println("Backing Up"); Serial1.println("Backing Up");
    Serial.print("Y-Values: ");    Serial.println(yVal);  Serial1.print("Y-Values: ");    Serial1.println(yVal);
  } else if (xVal > 0) {   //If the xVal is more than 0, turn right
    enableMotor(BOTH_MOTORS);   //Turns on both motors
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);   //Sets the left motor to forward
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD); //Sets the right motor to backward
    setMotorSpeed(BOTH_MOTORS, xVal);                   //Sets both motors speed to xVal
    Serial.println("Turning Right"); Serial1.println("Turning Right");
    Serial.print("X-Values: "); Serial.println(xVal); Serial1.print("X-Values: "); Serial1.println(xVal);
  } else if (xVal < 0) {   //If the xVal is less than 0, turn left
    enableMotor(BOTH_MOTORS);   //Turns on both motors
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);  //Sets the left motor to backward
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);  //Sets the right motor to forward
    setMotorSpeed(BOTH_MOTORS, xVal * -1);                   //Sets both motors speed to xVal
    Serial.println("Turning Left"); Serial1.println("Turning Left");
    Serial.print("X-Values: "); Serial.println(xVal); Serial1.print("X-Values: "); Serial1.println(xVal);
  } else if (ps2x.ButtonPressed(PSB_START)) {
    STATE = AUTO;
  } else if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
    STATE = CLOSE;
  } else if (ps2x.ButtonPressed(PSB_SQUARE)) {
    STATE = OPEN;
  } else if (ps2x.ButtonPressed(PSB_SELECT)) {
    STATE = SONG;
  }
  else {
    disableMotor(BOTH_MOTORS);    //Disables both of the motors if both xVal = 0
  }
}
//Code to shoot monsters, health, and weapon pack
void IR() {
  //When circle is pressed, it shoots 
  if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    IRHlP.protocol = NEC;
    IRHlP.address = 160;
    IRHlP.command = 123;
    IRHlP.isRepeat = false;
    sendIR.write(&IRHlP);
    delay(500);
  }
  //When X is pressed, it shoots 
  if (ps2x.ButtonPressed(PSB_CROSS)) {
    IRWnP.protocol = NEC;
    IRWnP.address = 160;
    IRWnP.command = 143;
    IRWnP.isRepeat = false;
    sendIR.write(&IRWnP);
    delay(500);
  }
}
//Follow the white line autonomously
void autonomousState() {
  senseLight();     //Runs the light function
  Serial.println("Now in autonomous."); Serial1.println("Now in autonomous.");
  lineColor = LIGHT_LINE;
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, lineColor);
  linePos = getLinePosition(sensorCalVal, lineColor);
  enableMotor(BOTH_MOTORS);
  Serial.println(sensorCalVal[7]);
  rightsensval = sensorCalVal[7];
  Serial.println(sensorCalVal[0]);
  leftsensval = sensorCalVal[0];
  if (rightsensval > 20 && leftsensval > 20) {
    disableMotor(BOTH_MOTORS);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
    delayMicroseconds(1200 * MS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    delayMicroseconds(500 * MS);
    gripper.write(20);
    delayMicroseconds(10 * MS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
  } else if ( rightsensval > 20) {
    disableMotor(BOTH_MOTORS);
    delayMicroseconds(1 * MS);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
    delayMicroseconds(20 * MS);
    disableMotor(BOTH_MOTORS);
  } else if ( leftsensval > 20) {
    disableMotor(BOTH_MOTORS);
    delayMicroseconds(1 * MS);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
    delayMicroseconds(20 * MS);
    disableMotor(BOTH_MOTORS);
  } else if (linePos > 0 && linePos < 3000) {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  } else if(linePos > 3500) {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
  } else {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
  }
  if (ps2x.ButtonPressed(PSB_START)) {
    disableMotor(BOTH_MOTORS);
    STATE = MOVE;
  } else if(ps2x.ButtonPressed(PSB_TRIANGLE)){
    disableMotor(BOTH_MOTORS);
    STATE = CLOSE;
  }  else if(ps2x.ButtonPressed(PSB_SQUARE)){
    disableMotor(BOTH_MOTORS);
    STATE = OPEN;
  } else if(ps2x.ButtonPressed(PSB_SELECT)){
    STATE = SONG;
  } else {
    STATE = AUTO;
  }
}
//Calibrates the sensor on the bottom of the robot to the floor
void simpleCalibrate() {
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD); //Set both motors direction forward
  enableMotor(BOTH_MOTORS);         //Enable both motors
  setMotorSpeed(BOTH_MOTORS, 20);   //Sets both motors speed to 20
  for (int x = 0; x < 100; x++) {
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
    readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, 0);
    linePosition = getLinePosition(sensorCalVal, lineMode);
  }
  disableMotor(BOTH_MOTORS);  //Disables both motors
}
//Prints out if 
void floorCalibration() {
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  delay(1000);
  Serial.println("Running calibration on floor");   Serial1.println("Running calibration on floor");
  simpleCalibrate();      //Runs the calibration function
  Serial.println("Reading floor values complete");    Serial1.println("Reading floor values complete");
  delay(1000);
  enableMotor(BOTH_MOTORS);
}

//Senses the light value
void senseLight() {
  light = analogRead(photores);
  if (light < 150) {
    digitalWrite(led, HIGH);
    delayMicroseconds(100 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(100 * MS);
    digitalWrite(led, HIGH);
    delayMicroseconds(100 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(100 * MS);
    digitalWrite(led, HIGH);
    delayMicroseconds(100 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(100 * MS);

    digitalWrite(led, HIGH);
    delayMicroseconds(200 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(200 * MS);
    digitalWrite(led, HIGH);
    delayMicroseconds(200 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(200 * MS);
    digitalWrite(led, HIGH);
    delayMicroseconds(200 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(200 * MS);

    digitalWrite(led, HIGH);
    delayMicroseconds(100 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(100 * MS);
    digitalWrite(led, HIGH);
    delayMicroseconds(100 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(100 * MS);
    digitalWrite(led, HIGH);
    delayMicroseconds(100 * MS);
    digitalWrite(led, LOW);
    delayMicroseconds(100 * MS);
  } else {
    digitalWrite(led, LOW);
  }
}
//Plays the default dance song on buzzer when select is pressed
void playSong() {
  playNote("g4", "eighth");
  restNote("eighth");
  restNote("quarter");
  playNote("g4", "quarter");
  restNote("sixteenth");
  playNote("b4", "quarter");
  restNote("sixteenth");
  playNote("c5", "half");
  restNote("sixteenth");
  playNote("c5", "quarter");
  restNote("sixteenth");
  playNote("b4", "quarter");
  restNote("whole");
  playNote("g4", "quarter");
  restNote("sixteenth");
  playNote("b4", "quarter");
  restNote("sixteenth");
  playNote("c5", "half");
  restNote("sixteenth");
  playNote("c5", "eighth");
  restNote("sixteenth");
  playNote("b4", "quarter");
  restNote("sixteenth");
  playNote("g4", "quarter");
  restNote("sixteenth");
  playNote("f4", "eighth");
  restNote("sixteenth");
  playNote("g4", "quarter");
  restNote("eighth");
  restNote("quarter");
  playNote("c5", "eighth");
  restNote("sixteenth");
  playNote("b4", "quarter");
  restNote("sixteenth");
  playNote("g4", "quarter");
  restNote("sixteenth");
  playNote("f4", "quarter");
  restNote("sixteenth");
  playNote("g4", "quarter");
  playNote("g4", "eighth");
  restNote("eighth");
  restNote("quarter");
  restNote("half");
  Serial1.println(noteSpeed);
  STATE = IDLE;
}
// function used to set Buzzer Pin to a new pin number
void setBuzzerPin(int newBuzzerPin) {
  buzzerPin = newBuzzerPin;
  pinMode(buzzerPin, OUTPUT); // Make pin 34 an output
}
// function to play a rest note with a certain type (whole, half, quarter, eighth, and sixteenth)
void restNote(String restType) {
  if (restType == "whole") {
    Serial1.println("Rest Length: whole");
    restLength = (int)(4000.0 * noteSpeed);
  }
  else if (restType == "half") {
    Serial1.println("Rest Length: half");
    restLength = (int)(2000.0 * noteSpeed);
  }
  else if (restType == "quarter") {
    Serial1.println("Rest Length: quarter");
    restLength = (int)(1000.0 * noteSpeed);
  }
  else if (restType == "eighth") {
    Serial1.println("Rest Length: eighth");
    restLength = (int)(500.0 * noteSpeed);
  }
  else if (restType == "sixteenth") {
    Serial1.println("Rest Length: sixteenth");
    restLength = (int)(250.0 * noteSpeed);
  }
  else {
    Serial1.println("Invalid Rest Type");
  }
  noTone(buzzerPin);
  delayMicroseconds(restLength * MS);
}
// function to play a note between C8 and C1 (NOTE: No rests are added between notes, they must be added manually)
void playNote(String note, String noteType) {
  if (noteType == "whole") {
    Serial1.println("Note Length: whole");
    noteLength = (int)(4000.0 * noteSpeed);
  }
  else if (noteType == "half") {
    Serial1.println("Note Length: half");
    noteLength = (int)(2000.0 * noteSpeed);
  }
  else if (noteType == "quarter") {
    Serial1.println("Note Length: quarter");
    noteLength = (int)(1000.0 * noteSpeed);
  }
  else if (noteType == "eighth") {
    Serial1.println("Note Length: eighth");
    noteLength = (int)(500.0 * noteSpeed);
  }
  else if (noteType == "sixteenth") {
    Serial1.println("Note Length: sixteenth");
    noteLength = (int)(250.0 * noteSpeed);
  }
  else {
    Serial1.println("Invalid Note Type");
  }
  if (note == "c1") {
    Serial1.println("Playing c1");
    tone(buzzerPin, 33);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "d1") {
    Serial1.println("Playing d1");
    tone(buzzerPin, 37);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "e1") {
    Serial1.println("Playing e1");
    tone(buzzerPin, 41);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "f1") {
    Serial1.println("Playing f1");
    tone(buzzerPin, 44);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "g1") {
    Serial1.println("Playing g1");
    tone(buzzerPin, 49);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "a1") {
    Serial1.println("Playing a1");
    tone(buzzerPin, 55);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "b1") {
    Serial1.println("Playing b1");
    tone(buzzerPin, 62);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "c2") {
    Serial1.println("Playing c2");
    tone(buzzerPin, 65);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "d2") {
    Serial1.println("Playing d2");
    tone(buzzerPin, 73);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "e2") {
    Serial1.println("Playing e2");
    tone(buzzerPin, 82);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "f2") {
    Serial1.println("Playing f2");
    tone(buzzerPin, 87);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "g2") {
    Serial1.println("Playing g2");
    tone(buzzerPin, 98);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "a2") {
    Serial1.println("Playing a2");
    tone(buzzerPin, 110);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "b2") {
    Serial1.println("Playing b2");
    tone(buzzerPin, 124);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "c3") {
    Serial1.println("Playing c3");
    tone(buzzerPin, 131);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "d3") {
    Serial1.println("Playing d3");
    tone(buzzerPin, 147);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "e3") {
    Serial1.println("Playing e3");
    tone(buzzerPin, 165);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "f3") {
    Serial1.println("Playing f3");
    tone(buzzerPin, 175);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "g3") {
    Serial1.println("Playing g3");
    tone(buzzerPin, 196);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "a3") {
    Serial1.println("Playing a3");
    tone(buzzerPin, 220);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "b3") {
    Serial1.println("Playing b3");
    tone(buzzerPin, 247);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "c4") {
    Serial1.println("Playing c4");
    tone(buzzerPin, 262);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "d4") {
    Serial1.println("Playing d4");
    tone(buzzerPin, 294);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "e4") {
    Serial1.println("Playing e4");
    tone(buzzerPin, 330);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "f4") {
    Serial1.println("Playing f4");
    tone(buzzerPin, 349);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "g4") {
    Serial1.println("Playing g4");
    tone(buzzerPin, 392);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "a4") {
    Serial1.println("Playing a4");
    tone(buzzerPin, 440);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "b4") {
    Serial1.println("Playing b4");
    tone(buzzerPin, 494);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "c5") {
    Serial1.println("Playing c5");
    tone(buzzerPin, 523);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "d5") {
    Serial1.println("Playing d5");
    tone(buzzerPin, 587);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "e5") {
    Serial1.println("Playing e5");
    tone(buzzerPin, 659);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "f5") {
    Serial1.println("Playing f5");
    tone(buzzerPin, 699);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "g5") {
    Serial1.println("Playing g5");
    tone(buzzerPin, 784);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "a5") {
    Serial1.println("Playing a5");
    tone(buzzerPin, 880);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "b5") {
    Serial1.println("Playing b5");
    tone(buzzerPin, 988);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "c6") {
    Serial1.println("Playing c6");
    tone(buzzerPin, 1047);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "d6") {
    Serial1.println("Playing d6");
    tone(buzzerPin, 1175);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "e6") {
    Serial1.println("Playing e6");
    tone(buzzerPin, 1319);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "f6") {
    Serial1.println("Playing f6");
    tone(buzzerPin, 1397);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "g6") {
    Serial1.println("Playing g6");
    tone(buzzerPin, 1568);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "a6") {
    Serial1.println("Playing a6");
    tone(buzzerPin, 1760);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "b6") {
    Serial1.println("Playing b6");
    tone(buzzerPin, 1980);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "c7") {
    Serial1.println("Playing c7");
    tone(buzzerPin, 2093);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "d7") {
    Serial1.println("Playing d7");
    tone(buzzerPin, 2349);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "e7") {
    Serial1.println("Playing e7");
    tone(buzzerPin, 2637);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "f7") {
    Serial1.println("Playing f7");
    tone(buzzerPin, 2794);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "g7") {
    Serial1.println("Playing g7");
    tone(buzzerPin, 3136);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "a7") {
    Serial1.println("Playing a7");
    tone(buzzerPin, 3520);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "b7") {
    Serial1.println("Playing b7");
    tone(buzzerPin, 3951);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "c8") {
    Serial1.println("Playing c8");
    tone(buzzerPin, 4186);
    delayMicroseconds(noteLength * MS);
  }
}
//Closes the gripper
void closeState() {
  gripper.write(95);              //Closes gripper to 30 degrees
  Serial.println("Close Claw"); Serial1.println("Close Claw");  //Prints indicator to serial monitor
  delayMicroseconds(10 * MS);
  STATE = IDLE;                   //Returns the state to IDLE
}
//Opens the gripper
void openState() {
  gripper.write(20);           //Opens gripper to 160 degrees
  Serial.println("Open Claw"); Serial1.println("Open Claw"); //Prints indicator to serial monitor
  delayMicroseconds(10 * MS);
  STATE = IDLE;                 //Returns the state to IDLE
}
//Reads what button is pressed then goes into the respective state.
void idleState() {
  Serial1.println("IDLE STATE");
  if (ps2x.ButtonPressed(PSB_START)) {
    STATE = AUTO;
  } else if (ps2x.ButtonPressed(PSB_SQUARE))  {        //If square button is pressed...
    STATE = OPEN;     //Changes state to OPEN
  } else if (ps2x.ButtonPressed(PSB_TRIANGLE)) { //If triangle button is pressed...
    STATE = CLOSE;    //Changes state to CLOSE
  } else if (ps2x.ButtonPressed(PSB_SELECT)) {
    STATE = SONG;
  }
  else {
    STATE = MOVE;     //Restarts the IdleState function until a button is pressed/read by robot
  }
}
