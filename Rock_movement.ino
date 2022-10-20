/* Code to get robot moving with the PS joysticks.
 * 
 */

// PS2 Controller Constants
#include "PS2X_lib.h"  //for v1.6  //Adds PS2 controller library
#include "SimpleRSLK.h"
#define PS2_DAT   14   //P1.7 <-> brown wire
#define PS2_CMD   15   //P1.6 <-> orange wire
#define PS2_SEL   34   //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK   35   //P6.7 <-> blue wire
PS2X ps2x;             // create PS2 Controller Class

//Robot movement variables
#include "SimpleRSLK.h"                 //Robot library
uint16_t sensorVal[LS_NUM_SENSORS];     
uint16_t sensorCalVal[LS_NUM_SENSORS];  
uint16_t sensorMaxVal[LS_NUM_SENSORS];  
uint16_t sensorMinVal[LS_NUM_SENSORS];  

//DC variables
int speedsetting=0;
float dutycycle=0;

//Defines the states
#define STOP    0
#define GO      1
int STATE = STOP;
int yneg = 0;
int ypos = 0;

void setup() {
  Serial.begin(57600);            //Prints to serial monitor while usb is connected
  Serial1.begin(57600);           //Prints to serial monitor through bluetooth
  delayMicroseconds(500 * 1000);  //added delay to give wireless ps2 module some time to startup
  PS2ControllerSetup();
  setupRSLK();                    //Sets the robot wheel pins
}

void loop() {
    PS2ButtonDetect();
    switch (STATE) {
      case STOP:
        Serial.println("I am not moving.");
        noSchmoove();
        break;
      case GO:
        Serial.println("I am moving.");
        Schmoove();
        break;
    }
}

void PS2ControllerSetup(){
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?)
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
}

void PS2ButtonDetect() {
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

void Schmoove() {
//Forward
  if(ypos>512){
    setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD)
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS,50);
  }
  if(yneg<512){
    setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD)
  }
}
void noSchmoove() {
  dutyc=75;
  speedsetting = dutyc / 100 * 255;
}
