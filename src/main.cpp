#include <Arduino.h>
#include "SPI.h"
#include <Wire.h>
#include <OneBitDisplay.h>
#include "header_total.hpp"
#include <vector>

#define ROVER 2;

float counter = 0;

hw_timer_t * timer = NULL;

volatile bool sample = false;

void ARDUINO_ISR_ATTR onTimer(){
  sample = true;
  counter = counter + 1;
}

OBDISP obd; // OneBitDisplay library init
OpticalSensor os = OpticalSensor();
DriveController rover = DriveController(os);

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

  interrupts();

}
// float display_error; 


// void update_oled() {

//   char szTemp[32];
//   obdFill(&obd, 0x0, 1);

//   sprintf(szTemp, "Err : %f", display_error);   obdWriteString(&obd, 0,0,0,(char *)szTemp, FONT_12x16, 1, 1);
//   sprintf(szTemp, "Speed : %d", (int)speed);      obdWriteString(&obd, 0,0,22,(char *)szTemp, FONT_8x8, 0, 1);
//   sprintf(szTemp, "Left  : %d", (int)left_speed);  obdWriteString(&obd, 0,0,30,(char *)szTemp, FONT_8x8, 0, 1);
//   sprintf(szTemp, "Right : %d", (int)right_speed); obdWriteString(&obd, 0,0,38,(char *)szTemp, FONT_8x8, 0, 1);

//   if (translate || !rotate) {
//     sprintf(szTemp, "Dist  : %d", (int)(total_y - initial_y));      obdWriteString(&obd, 0,0,46,(char *)szTemp, FONT_8x8, 0, 1);
//   } else if (rotate) {
//     sprintf(szTemp, "Angle : %d", (int)angle);      obdWriteString(&obd, 0,0,46,(char *)szTemp, FONT_8x8, 0, 1);
//   }
//   if      (translate)     obdWriteString(&obd, 0,0,52,(char *)"Translating...", FONT_8x8, 0, 1);
//   else if (rotate)    obdWriteString(&obd, 0,0,52,(char *)"Rotating...", FONT_8x8, 0, 1);
//   else obdWriteString(&obd, 0,0,52,(char *)"Doing nothing...", FONT_8x8, 0, 1);
// }



void loop() {

  float speed = 60;

  // // Update the OLED - see function for what it displays
  // if (counter >= 100) {
  //     update_oled();
  //     counter = 0;
  // }

  rover.Run(sample);
  
}