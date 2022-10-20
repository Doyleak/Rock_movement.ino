

#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>

#include "PS2X_lib.h"  //for v1.6
#include <Servo.h>

/*

   PS2 Controller example from the PS2_Lib
   2021-10-08: Refactored for MSP432P401R ZJE
               -Changed pins to good defaults for use with RSLK
               -changed all delay() to delayMicroseconds

   Original code: https://github.com/madsci1016/Arduino-PS2X
   Documentation: http://www.billporter.info/2010/06/05/playstation-2-controller-arduino-library-v1-0/

   If you are using the PS2 bluetooth dongle, ensure that your
   controller turned on before the program starts

*/


/******************************************************************
   set pins connected to PS2 controller:
     - 1e column: original
     - 2e colmun: Stef?
   replace pin numbers by the ones you use
 ******************************************************************/
//ZJE: Modified from Bill's example to use MSP432P401R pins not taken by the
//RSLK Feel free to use other pins that not used by the RSLK, these were just
//chosen for neatness
#define PS2_DAT         14 //P1.7 <-> brown wire
#define PS2_CMD         15 //P1.6 <-> orange wire
#define PS2_SEL         34 //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK         35 //P6.7 <-> blue wire
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

/******************************************************************
   select modes of PS2 controller:
     - pressures = analog reading of push-butttons
     - rumble    = motor rumbling
   uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class
Servo gripper;

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your board after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

#define IDLE 4
#define OPEN 5
#define CLOSE 6

int STATE = IDLE;
#define MS 1000

void setup() {

  Serial.begin(57600); //ZJE: changed from Arduino deafult of 9600
  gripper.attach(SRV_0);
  gripper.write(5);
  Serial.println("angle = 5");
  delayMicroseconds(500 * 1000); //added delay to give wireless ps2 module some time to startup, before configuring it

  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

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
      Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
      Serial.println("holding L1 or R1 will print out the analog stick values.");
      Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
    }  else if (error == 1)
      Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

    else if (error == 2)
      Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

    else if (error == 3)
      Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    delayMicroseconds(1000 * 1000);
  }


  //  Serial.print(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
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

    default:
      break;
  }

}

void CloseState() {
    gripper.write(30);
    Serial.println("angle = 30");
    delayMicroseconds(1000 * MS); //can’t use delay
  
  STATE = IDLE;

}
void OpenState() {
    gripper.write(160);
    Serial.println("angle = 160");
    delayMicroseconds(1000 * MS); //can’t use delay
  
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
    STATE = CLOSE;
  }
  else {
    STATE = OPEN;
  }
}
void IdleState() {
  if (ps2x.ButtonPressed(PSB_SQUARE))  {           //will be TRUE if button was JUST released
    STATE = OPEN;
  }
  else {
    STATE = IDLE;
  }
}
/* You must Read Gamepad to get new values and set vibration values
   ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
   if you don't enable the rumble, use ps2x.read_gamepad(); with no values
   You should call this at least once a second
*/
void detect() {
  if (error == 1) { //skip loop if no controller found
    return;
  }

  if (type != 2) { //DualShock or Wireless DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
  }

  if (ps2x.Button(PSB_PAD_UP)) {
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
  }
  if (ps2x.Button(PSB_PAD_RIGHT)) {
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
  }
  if (ps2x.Button(PSB_PAD_LEFT)) {
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
  }
  if (ps2x.Button(PSB_PAD_DOWN)) {
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
  }

  vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
  if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
    if (ps2x.ButtonPressed(PSB_TRIANGLE))
      Serial.println("Triangle pressed");
  }

  if (ps2x.ButtonPressed(PSB_CIRCLE))              //will be TRUE if button was JUST pressed
    Serial.println("Circle just pressed");
  if (ps2x.ButtonPressed(PSB_CROSS))              //will be TRUE if button was JUST pressed OR released
    Serial.println("X just changed");
  if (ps2x.ButtonPressed(PSB_SQUARE))             //will be TRUE if button was JUST released
    Serial.println("Square just released");


  delayMicroseconds(50 * 1000);
}
