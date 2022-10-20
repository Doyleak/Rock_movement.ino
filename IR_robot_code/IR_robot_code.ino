#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>

#include <TinyIR.h>



#define MS 1000

int IRpin = 33;


IRData IRresults;


void setup() {
  Serial.println("Hello World");
  Serial.begin(57600);
  delayMicroseconds(100*MS);
  initTinyIRReceiver();

  pinMode(IRpin, INPUT);

  setupRSLK();

  
}

void loop() {

  uint16_t normalSpeed = 10;
  enableMotor(BOTH_MOTORS);


 if(decodeIR(&IRresults)){
    Serial.print(IRresults.command, HEX);
    translateIR();
  }
  
}

void translateIR(){ 
  uint16_t normalSpeed = 10;
  Serial.print("translate IR: ");
  switch(IRresults.command){
    case 0x45:
      Serial.println("POWER");
      break;
    case 0x46:
      if(IRresults.isRepeat){               //ignores repeated button inputs
        Serial.println( );
      }
      else{
      Serial.println("VOL+");
      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR,normalSpeed);
      setMotorSpeed(RIGHT_MOTOR,normalSpeed);
 
      }  
      break;
      
    case 0x47:
      Serial.println("FUNC");
      break;
    case 0x44:
      Serial.println("LEFT");
      break;
    case 0x40:
      Serial.println("PLAY");
      break;
    case 0x43:
      Serial.println("RIGHT");
      break;
    case 0x9:
      Serial.println("UP");
      break;
    case 0x15:
    
      if(IRresults.isRepeat){               //ignores repeat but button press (cannot hold down button)
      Serial.println( );
      }
      else{
       Serial.println("VOL-");
       enableMotor(BOTH_MOTORS);
       setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);
       setMotorSpeed(LEFT_MOTOR,normalSpeed);
       setMotorSpeed(RIGHT_MOTOR,normalSpeed);
      }
      break;
      
    case 0x7:
      Serial.println("DOWN");
      break;
    case 0x16:
      Serial.println("0");
      break;
    case 0x19:
      Serial.println("EQ");
      break;
    case 0xD:
      Serial.println("ST");
      break;
    case 0xC:
      Serial.println("1");
      break;
    case 0x18:
      Serial.println("2");
      break;
    case 0x5E:
      Serial.println("3");
      break;
    case 0x8:
      Serial.println("4");
      break;
    case 0x1C:
      Serial.println("5");
      break;
    case 0x5A:
      Serial.println("6");
      break;
    case 0x42:
      Serial.println("7");
      break;
    case 0x52:
      Serial.println("8");
      break;
    case 0x4A:
      Serial.println("9");
      break;
    default:
      Serial.println("other button");
      break;
  }
  delayMicroseconds(100*MS);
}
