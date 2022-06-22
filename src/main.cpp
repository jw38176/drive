#include <Arduino.h>
#include "SPI.h"
#include <Wire.h>
#include "header_total.hpp"
#include <vector>
#include "drive.hpp"
#include <OneBitDisplay.h>

#define ROVER 2;

float counter = 0;

hw_timer_t * timer = NULL;

volatile bool sample = false;

void ARDUINO_ISR_ATTR onTimer(){
  sample = true;
  counter = counter + 1;
}

OBDISP obd; // OneBitDisplay library init
OpticalSensor os;
DriveController rover;


//DriveController::RoverStates states;

void setup() {

  Serial.begin(9600);
  millis();
  delay(3000);

  noInterrupts();

  int rc = obdI2CInit(&obd, OLED_128x64, -1, 0, 0, 1, -1, -1, -1, 800000L); // OLED display setup

  //code to interrupt every 2ms
  timer = timerBegin(0,80,true);
  timerAttachInterrupt(timer,&onTimer,true);
  timerAlarmWrite(timer,2000,true);
  timerAlarmEnable(timer);

  os.InitSensor();
  rover.InitController(os);

  interrupts();

  float set_des_speed = 60;
  float set_des_angle = 90;
  float set_des_trans = 100;

  rover.SetAngle(set_des_angle);
  rover.SetSpeed(set_des_speed);
  rover.SetTrans(set_des_trans);
  rover.ChangeState(rover.ROTATE);

}

float display_error; 

void loop() {

  rover.Run(sample);
  if(rover.current_roverstate == rover.IDLE){
    rover.ChangeState(rover.MOVE);
  }

}