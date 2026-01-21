#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "CommandHandler.h"

static int servo_angles[16] = {80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80};
int i = 0;
static int k[1];

CommandHandler<10, 90, 15> SerialCommandHandler;


const int SERVOMIN = 125;
const int SERVOMAX = 575;
const int SERVONUM = 16;

static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void updateAllServosAngle(CommandParameter &parameters){

    int ang0 = parameters.NextParameterAsInteger();
    servo_angles[k[0]] = ang0;
    int ang1 = parameters.NextParameterAsInteger();
    servo_angles[k[0] + 1] = ang1;
    int ang2 = parameters.NextParameterAsInteger();
    servo_angles[k[0] + 2] = ang2;
    int ang3 = parameters.NextParameterAsInteger();
    servo_angles[k[0] + 3] = ang3;

    
}

void updateMode (CommandParameter &parameters){

  int k_val = parameters.NextParameterAsInteger();
  Serial.print(k_val);
  k[0] = k_val;

}


void conn(CommandParameter &parameters){

    Serial.print(F("Connection confirmed and Motors Started!")); // When Connected Return to Python initiating the Motors
   
    
}

void setup() {
  Serial.begin(250000);
 
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm.setOscillatorFrequency(27000000);
  // pwm.setPWM(0, 0, SERVOMAX);


  SerialCommandHandler.AddCommand(F("connect"), conn);
  SerialCommandHandler.AddCommand(F("UA"), updateAllServosAngle);
  SerialCommandHandler.AddCommand(F("UK"), updateMode);


}
int angleToPulse(int ang){
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}
int angleToPulseinv(int ang){
  int pulse = map(ang, 180, 0, SERVOMIN, SERVOMAX);
  return pulse;
}


void setAllServosAngle(){


  pwm.setPWM(0+k[0],0,angleToPulseinv(servo_angles[0+k[0]]));
  pwm.setPWM(1+k[0],0,angleToPulseinv(servo_angles[1+k[0]]));
  pwm.setPWM(2+k[0],0,angleToPulseinv(servo_angles[2+k[0]]));
  pwm.setPWM(3+k[0],0,angleToPulse(servo_angles[3+k[0]]));

}
    


void loop() {
  SerialCommandHandler.Process();
  setAllServosAngle();
}



