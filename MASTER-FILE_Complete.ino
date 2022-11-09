/*  This is the final completed code for our robot.
    This makes the robot move with the playstation controller, makes the robot move by itself, 
    calibrate it's position on the floor, and play a little song we made.
    Date: 11/9/22
    Author:Abigail Doyle
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
#define PS2_DAT       14    //P1.7 = brown wire
#define PS2_CMD       15    //P1.6 = orange wire
#define PS2_SEL       34    //P2.3 = yellow wire (attention)
#define PS2_CLK       35    //P6.7 = blue wire
#define PS2X_DEBUG          //Controller debug
#define PS2X_COM_DEBUG      //Controller debug

//All the cases used in switch(STATE)
#define MOVE 1      //The move case
#define AUTO 2      //The auto case
#define SONG 3      //The song case

#define MS 1000                 //Variable for changing the milliseconds to seconds
#define pressures   false       //Not using pressure sensors on controller
#define rumble      false       //Not using rumble on controller
#define IR_TRX_PIN 17           //Sets the IR pin to 17
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

//Speed variables
uint16_t normalSpeed = 25;  //Sets normal speed variable to 25
uint16_t fastSpeed = 33;    //Sets normal speed variable to 33
uint16_t turnSpeed = 40;    //Sets normal speed variable to 40
//Line following variables
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint8_t lineMode = 0;
uint8_t lineColor;          //Variable for line color
uint32_t linePos;           //Variable for line position

int xVal, yVal;       //Variables for x-values and y-values
int STATE = MOVE;     //Starts the robot in the MOVE case
int rightsensval;     //Variable for the right floor sensor
int leftsensval;      //Variable for the left floor sensor
int linePosition = 0; //Sets line position to 0.
int photores = 33;    //Sets the photores variable to 33
int blueLED = 2;      //The blue LED pin = 2
int light = 0;        //The starting light value = 0
int Health, Weapon;   //Health and Weapons variables

bool isCalibrationComplete = false;   //Sets the calibration to false when the program starts.

PS2X ps2x;        //PS2 Controller Class and object
Servo gripper;    //Declares Gripper as the servo object
IRData IRHlP;     //Health Pack
IRData IRWnP;     //Weapons Pack
IRsend sendIR;    //Sets sendIR as an object

//Fortnite Buzzer Constants
int buzzerPin = 37;         // buzzer pin is not assigned to anything specific until setup, just set to 0
int noteLength = 0;         // noteLength assigned to default value, isn't used
int restLength = 0;         // restLength assigned to default value, isn't used
float noteSpeed = 0.125;    // speed of the notes/rests.

void setup() {
  Serial1.begin(57600);  //Sets the baud rate to read and print to the serial monitor.
  gripper.attach(SRV_0);        //Initializes the gripper servo
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);
  pinMode(blueLED, OUTPUT);
  pinMode(photores, INPUT);
  setupRSLK();        //Sets up the DC motor pins (aka the wheel pins)
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);   //Sets up pins and settings for the PS Controller
  delayMicroseconds(1000 * MS);       //Delay for a full second
  sendIR.begin(IR_TRX_PIN, true, GREEN_LED);
}

//Repeats endlessly until robot is turned off or loses power.
void loop() {
  ps2x.read_gamepad();      //Constantly reads the Playstation controller buttons
//If calibration isn't complete, then it runs the floor calibration, then goes into the MOVE state.
  if (isCalibrationComplete == false) {  //If floor calibration isn't complete...
    floorCalibration();         //then run the floorCalibration function
    isCalibrationComplete = true;     //Once the floor calibration is complete, it sets calibration to true
    STATE = MOVE;       //Sets the state to MOVE.
  }
  switch (STATE) {
    case MOVE:
      movement();
      break;
    case AUTO:
      autonomousState();
      break;
    case SONG:
      playSong();
      break;
    default:
      break;
  }
}

//Function that runs the floor calibration.
void floorCalibration() {
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  delay(1000);
  Serial1.println("Running floor calibration...");
//Calibration code
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
  Serial1.println("Calibration complete");
  delay(1000);
}

//This function reads the joystick values and makes the robot move.
void movement() {
  delayMicroseconds(50 * MS);
  xVal = ps2x.Analog(PSS_RX);           //Assigns the variable xVal, to the right X joystick values
  xVal = map(xVal, 128, 255, 0, 20);    //Changes the xVal from 0-255 to 0-40
  yVal = ps2x.Analog(PSS_LY);           //Assigns the variable yVal, to the left Y joystick values
  yVal = map(yVal, 127, 255, 0, 50);    //Changes the yVal from 0-255 to 0-40
  if (yVal < 0) {          //If the yVal is less than 0, go forward
    Serial1.println("Going Forward");
    Serial1.print("Y-Values: ");    Serial1.println(yVal);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);  //Sets both motors to forward
    enableMotor(BOTH_MOTORS);   //Turns on both motors
    setMotorSpeed(BOTH_MOTORS, yVal * -1);                   //Sets motor speed to yVal.
  } else if (yVal > 0) {   //If the yVal is less than 0, go backward
    Serial1.println("Backing Up");
    Serial1.print("Y-Values: ");    Serial1.println(yVal);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD); //Sets both motors to backward
    enableMotor(BOTH_MOTORS);   //Turns on both motors
    setMotorSpeed(BOTH_MOTORS, yVal);                   //Sets motor speed to yVal
  } else if (xVal > 0) {   //If the xVal is more than 0, turn right
    Serial1.println("Turning Right");
    Serial1.print("X-Values: "); Serial1.println(xVal);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);   //Sets the left motor to forward
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD); //Sets the right motor to backward
    enableMotor(BOTH_MOTORS);   //Turns on both motors
    setMotorSpeed(BOTH_MOTORS, xVal);                   //Sets both motors speed to xVal
  } else if (xVal < 0) {   //If the xVal is less than 0, turn left
    Serial1.println("Turning Left");
    Serial1.print("X-Values: "); Serial1.println(xVal);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);  //Sets the left motor to backward
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);  //Sets the right motor to forward
    enableMotor(BOTH_MOTORS);   //Turns on both motors
    setMotorSpeed(BOTH_MOTORS, xVal * -1);                   //Sets both motors speed to xVal
  } else if (ps2x.ButtonPressed(PSB_START)) {       //If select button is pressed...
    Serial1.println("Mode = Autonomous");
    STATE = AUTO;       //Changes the state to AUTO
  } else if (ps2x.ButtonPressed(PSB_TRIANGLE)) {    //If triangle button is pressed...
    Serial1.println("Closing Claw");
    gripper.write(92);           //Closes gripper to 92 degrees
    delayMicroseconds(10 * MS);
  } else if (ps2x.ButtonPressed(PSB_SQUARE))  {     //If square button is pressed...
    Serial1.println("Opening Claw");
    gripper.write(20);           //Opens gripper to 20 degrees
    delayMicroseconds(10 * MS);
  } else if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    Serial1.println("Shooting Monster...");
    IRHlP.protocol = NEC;     //Sets up necessary conditions to transmit a signal (lines 188-191)
    IRHlP.address = 160;
    IRHlP.command = 123;      //IR code for health pack
    IRHlP.isRepeat = false;
    sendIR.write(&IRHlP);     //Transmits IR signal
    delay(500);
  } else if (ps2x.ButtonPressed(PSB_CROSS)) {
    Serial1.println("Shooting Health or weapon pack...");
    IRWnP.protocol = NEC;
    IRWnP.address = 160;
    IRWnP.command = 143;    //IR code for weapon pack and monsters
    IRWnP.isRepeat = false;
    sendIR.write(&IRWnP);
    delay(500);
  } else if (ps2x.ButtonPressed(PSB_SELECT)) {      //If select button is pressed...
    Serial1.println("Mode = Song");
    STATE = SONG;       //Changes the state to SONG
  } else {      //If all of the above isn't true, then...
    disableMotor(BOTH_MOTORS);
    STATE = MOVE;       //If none of the above buttons are pressed, then automatically restart the function.
  }
}

//Makes the robot move by itself, if the start button is pressed
void autonomousState() {
//Reads the light values
  light = analogRead(photores);
  Serial1.print("Light Value: ");  Serial1.println(light);
  if (light < 150) {      //If light is <150, then...
    digitalWrite(blueLED, HIGH);    //turn on the blue LED
  } else {      //If light is >150...
    digitalWrite(blueLED, LOW);     //turn off blue LED
  }
  lineColor = LIGHT_LINE;       //Sets the floor sensors to read the white line
  readLineSensor(sensorVal);    //Reads the line sensor values
  readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, lineColor);  //Reads values from the arrays
  linePos = getLinePosition(sensorCalVal, lineColor);     //Assigns getLinePosition to linePos
  enableMotor(BOTH_MOTORS);
  rightsensval = sensorCalVal[7];   //Assigns sensorCalVal 7 to rightsensval
  Serial1.print("Right Floor Value: ");  Serial1.print(rightsensval);
  leftsensval = sensorCalVal[0];    //Assigns sensorCalVal 0 to leftsensval
  Serial1.print(" || Left Floor Value: ");  Serial1.println(leftsensval);
  if (rightsensval > 20 && leftsensval > 20) {    //If left and right sensor values are > 20, then...
    disableMotor(BOTH_MOTORS);
  //Following code turns robot around
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);      //Sets the motor speed to 25
    delayMicroseconds(1000 * MS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    delayMicroseconds(50 * MS);
  //Opens gripper
    gripper.write(20);
    delayMicroseconds(10 * MS);
  //Moves robot forward
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
    delayMicroseconds(1 * MS);
  } else if(rightsensval > 20) {      //If right sensor values are greater than 20, then...
    disableMotor(BOTH_MOTORS);
    delayMicroseconds(1 * MS);
  //Turns robot to the right
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
    delayMicroseconds(20 * MS);
    disableMotor(BOTH_MOTORS);
  } else if(leftsensval > 20) {      //If left sensor values are greater than 20, then...
    disableMotor(BOTH_MOTORS);
    delayMicroseconds(1 * MS);
  //Turns robot to the left
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
    delayMicroseconds(20 * MS);
    disableMotor(BOTH_MOTORS);
  } else if(linePos > 0 && linePos < 3000) {    //If line is b/w 0 and 3000, then turn robot toward the line.
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);      //Sets the motor speed to 40
  } else if(linePos > 3500) {                   //If line is b/w 3500 and greater, then turn robot toward the line.
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
  } else {        //Makes the robot move straight ahead, if all of the above is false
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
  }
  if (ps2x.ButtonPressed(PSB_START)) {    //If start button is pressed, then...
    disableMotor(BOTH_MOTORS);
    Serial1.println("Mode = MOVE");
    STATE = MOVE;     //Change state to MOVE
  } else {            //If start button isn't pressed, then...
    STATE = AUTO;     //Restart the AUTO state
  }
}

//This plays the notes of the Fortnite song using playNote and restNote functions
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
  Serial1.print("Note Speed: ");  Serial1.println(noteSpeed);
  Serial1.println("Mode = MOVE");
  STATE = MOVE;     //Go back into the MOVE state.
}

//Plays a certain type of rest note: whole, half, quarter, eighth, or sixteenth
void restNote(String restType) {
  if (restType == "whole") {          //if the type of rest is a whole rest, then...
    Serial1.println("Rest Length: whole");
    restLength = (int)(4000.0 * noteSpeed);     //Make a rest by 4 seconds * note speed
  }
  else if (restType == "half") {      //if the type is a half rest, then...
    Serial1.println("Rest Length: half");
    restLength = (int)(2000.0 * noteSpeed);     //Make a rest by 2 seconds * note speed
  }
  else if (restType == "quarter") {   //if the type is a quarter rest, then...
    Serial1.println("Rest Length: quarter");
    restLength = (int)(1000.0 * noteSpeed);     //Make a rest by 1 second * note speed
  }
  else if (restType == "eighth") {    //if the type is a eighth rest, then...
    Serial1.println("Rest Length: eighth");
    restLength = (int)(500.0 * noteSpeed);      //Make a rest by .5 second * note speed
  }
  else if (restType == "sixteenth") { //if the type is a sixteenth rest, then...
    Serial1.println("Rest Length: sixteenth");
    restLength = (int)(250.0 * noteSpeed);      //Make a rest by .25 second * note speed
  }
  else {
    Serial1.println("Invalid Rest Type");
  }
  noTone(buzzerPin);
  delayMicroseconds(restLength * MS);
}

//Plays a note between C8 and C1 (NOTE: No rests are added between notes, they must be added manually)
void playNote(String note, String noteType) {
  if (noteType == "whole") {        //if the type is a whole note, then...
    Serial1.println("Note Length: whole");
    noteLength = (int)(4000.0 * noteSpeed);   //Make a note by 4 seconds * note speed
  }
  else if(noteType == "half") {     //if the type is a half note, then...
    Serial1.println("Note Length: half");
    noteLength = (int)(2000.0 * noteSpeed);   //Make a note by 2 seconds * note speed
  }
  else if(noteType == "quarter") {  //if the type is a quarter note, then...
    Serial1.println("Note Length: quarter");
    noteLength = (int)(1000.0 * noteSpeed);   //Make a note by 1 second * note speed
  }
  else if(noteType == "eighth") {   //if the type of rest is a whole rest, then...
    Serial1.println("Note Length: eighth");
    noteLength = (int)(500.0 * noteSpeed);    //Make a note by .5 seconds * note speed
  }
  else if(noteType == "sixteenth"){ //if the type of rest is a whole rest, then...
    Serial1.println("Note Length: sixteenth");
    noteLength = (int)(250.0 * noteSpeed);    //Make a note by .25 seconds * note speed
  }
  else {
    Serial1.println("Invalid Note Type");
  }
    /* series of if statements detect the note played between c1 and c8, 
     prints the note to the Serial Monitor, plays the note with the 
     appropriate frequency, and for the requested note duration */
  if (note == "c1") {
    Serial1.println("Playing c1");
    tone(buzzerPin, 33);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "d1") {
    Serial1.println("Playing d1");
    tone(buzzerPin, 37);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "e1") {
    Serial1.println("Playing e1");
    tone(buzzerPin, 41);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "f1") {
    Serial1.println("Playing f1");
    tone(buzzerPin, 44);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "g1") {
    Serial1.println("Playing g1");
    tone(buzzerPin, 49);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "a1") {
    Serial1.println("Playing a1");
    tone(buzzerPin, 55);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "b1") {
    Serial1.println("Playing b1");
    tone(buzzerPin, 62);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "c2") {
    Serial1.println("Playing c2");
    tone(buzzerPin, 65);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "d2") {
    Serial1.println("Playing d2");
    tone(buzzerPin, 73);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "e2") {
    Serial1.println("Playing e2");
    tone(buzzerPin, 82);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "f2") {
    Serial1.println("Playing f2");
    tone(buzzerPin, 87);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "g2") {
    Serial1.println("Playing g2");
    tone(buzzerPin, 98);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "a2") {
    Serial1.println("Playing a2");
    tone(buzzerPin, 110);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "b2") {
    Serial1.println("Playing b2");
    tone(buzzerPin, 124);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "c3") {
    Serial1.println("Playing c3");
    tone(buzzerPin, 131);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "d3") {
    Serial1.println("Playing d3");
    tone(buzzerPin, 147);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "e3") {
    Serial1.println("Playing e3");
    tone(buzzerPin, 165);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "f3") {
    Serial1.println("Playing f3");
    tone(buzzerPin, 175);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "g3") {
    Serial1.println("Playing g3");
    tone(buzzerPin, 196);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "a3") {
    Serial1.println("Playing a3");
    tone(buzzerPin, 220);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "b3") {
    Serial1.println("Playing b3");
    tone(buzzerPin, 247);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "c4") {
    Serial1.println("Playing c4");
    tone(buzzerPin, 262);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "d4") {
    Serial1.println("Playing d4");
    tone(buzzerPin, 294);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "e4") {
    Serial1.println("Playing e4");
    tone(buzzerPin, 330);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "f4") {
    Serial1.println("Playing f4");
    tone(buzzerPin, 349);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "g4") {
    Serial1.println("Playing g4");
    tone(buzzerPin, 392);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "a4") {
    Serial1.println("Playing a4");
    tone(buzzerPin, 440);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "b4") {
    Serial1.println("Playing b4");
    tone(buzzerPin, 494);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "c5") {
    Serial1.println("Playing c5");
    tone(buzzerPin, 523);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "d5") {
    Serial1.println("Playing d5");
    tone(buzzerPin, 587);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "e5") {
    Serial1.println("Playing e5");
    tone(buzzerPin, 659);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "f5") {
    Serial1.println("Playing f5");
    tone(buzzerPin, 699);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "g5") {
    Serial1.println("Playing g5");
    tone(buzzerPin, 784);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "a5") {
    Serial1.println("Playing a5");
    tone(buzzerPin, 880);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "b5") {
    Serial1.println("Playing b5");
    tone(buzzerPin, 988);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "c6") {
    Serial1.println("Playing c6");
    tone(buzzerPin, 1047);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "d6") {
    Serial1.println("Playing d6");
    tone(buzzerPin, 1175);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "e6") {
    Serial1.println("Playing e6");
    tone(buzzerPin, 1319);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "f6") {
    Serial1.println("Playing f6");
    tone(buzzerPin, 1397);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "g6") {
    Serial1.println("Playing g6");
    tone(buzzerPin, 1568);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "a6") {
    Serial1.println("Playing a6");
    tone(buzzerPin, 1760);
    delayMicroseconds(noteLength * MS);
  }
  else if (note == "b6") {
    Serial1.println("Playing b6");
    tone(buzzerPin, 1980);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "c7") {
    Serial1.println("Playing c7");
    tone(buzzerPin, 2093);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "d7") {
    Serial1.println("Playing d7");
    tone(buzzerPin, 2349);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "e7") {
    Serial1.println("Playing e7");
    tone(buzzerPin, 2637);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "f7") {
    Serial1.println("Playing f7");
    tone(buzzerPin, 2794);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "g7") {
    Serial1.println("Playing g7");
    tone(buzzerPin, 3136);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "a7") {
    Serial1.println("Playing a7");
    tone(buzzerPin, 3520);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "b7") {
    Serial1.println("Playing b7");
    tone(buzzerPin, 3951);
    delayMicroseconds(noteLength * MS);
  }
  else if(note == "c8") {
    Serial1.println("Playing c8");
    tone(buzzerPin, 4186);
    delayMicroseconds(noteLength * MS);
  }
}
