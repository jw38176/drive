#include <Arduino.h>
#include "SPI.h"
#include <Wire.h>
#include <OneBitDisplay.h>
#include "header_total.hpp"
#include <vector>
#include "drive.hpp"


#define ROVER 2;

OBDISP obd; // OneBitDisplay library init
OpticalSensor os;
DriveController rover;

float counter = 0;

hw_timer_t * timer = NULL;

volatile bool sample = false;

void ARDUINO_ISR_ATTR onTimer(){
  sample = true;
  counter = counter + 1;
}



std::vector<std::vector<String>> returnvect = rover.ReturnInfo();


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
  rover.ChangeState(rover.TRANSLATE);

  // if((rover.previous_roverstate == rover.TRANSLATE) && (rover.desired_angle>rover.current_angle)){
  //   rover.ChangeState(rover.ROTATE);
  // }
  // if((rover.previous_roverstate == rover.ROTATE)&&(rover.desired_translation>rover.current_y) ){
  //   rover.desired_translation = -rover.desired_translation;
  //   rover.ChangeState(rover.TRANSLATE);
  // }

}

float display_error; 


void update_oled() {

  char szTemp[32];
  obdFill(&obd, 0x0, 1);

  sprintf(szTemp, "Err : %f", display_error);   obdWriteString(&obd, 0,0,0,(char *)szTemp, FONT_12x16, 1, 1);
  sprintf(szTemp, "Speed : %d", (int)rover.speed);      obdWriteString(&obd, 0,0,22,(char *)szTemp, FONT_8x8, 0, 1);
  sprintf(szTemp, "Left  : %d", (int)rover.left_speed);  obdWriteString(&obd, 0,0,30,(char *)szTemp, FONT_8x8, 0, 1);
  sprintf(szTemp, "Right : %d", (int)rover.right_speed); obdWriteString(&obd, 0,0,38,(char *)szTemp, FONT_8x8, 0, 1);

  if (rover.current_roverstate == rover.TRANSLATE || !rover.current_roverstate == rover.ROTATE){
    sprintf(szTemp, "Dist  : %d", (int)(rover.current_y - rover.initial_y));      obdWriteString(&obd, 0,0,46,(char *)szTemp, FONT_8x8, 0, 1);
  } else if (rover.current_roverstate==rover.ROTATE){
    sprintf(szTemp, "Angle : %d", (int)rover.current_angle);      obdWriteString(&obd, 0,0,46,(char *)szTemp, FONT_8x8, 0, 1);
  }
  // if      (rover.TRANSLATE) obdWriteString(&obd, 0,0,52,(char *)"Translating...", FONT_8x8, 0, 1);
  // else if (rover.ROTATE)    obdWriteString(&obd, 0,0,52,(char *)"Rotating...", FONT_8x8, 0, 1);
  // else if (rover.IDLE) obdWriteString(&obd, 0,0,52,(char *)"Idle State...", FONT_8x8, 0, 1);


  //rover.ChangeState(rover.MOVE);  
}


void loop() {


  // if ((rover.previous_roverstate == rover.TRANSLATE)&&(rover.current_roverstate == rover.IDLE)) {
  //   rover.ChangeState(rover.ROTATE);
  // }

  if ((rover.previous_roverstate == rover.ROTATE) && (rover.current_roverstate == rover.IDLE)) {
    rover.SetAngle(-90);
    rover.SetSpeed(60);
    delay(2);
    rover.ChangeState(rover.ROTATE); 
  }
  if((rover.previous_roverstate == rover.TRANSLATE) && (rover.current_roverstate == rover.IDLE)){
    rover.SetTrans(-100);
    rover.SetSpeed(60);
    delay(20);
    rover.ChangeState(rover.TRANSLATE);
  }

  rover.Run(sample);
  if (counter == 300){
      Serial.print("( ");Serial.print(rover.x);Serial.print(" , ");Serial.print(rover.y);
      Serial.println(" )");
      Serial.print("heading: ");Serial.println(rover.heading);
      
      counter = 0;
    
  }

  

  returnvect = rover.ReturnInfo();

  // if (counter == 300){
  //   for (int i=0;i<returnvect.size();i++){
  //     for(int j=0;j<returnvect[i].size();j++){
  //       Serial.print(" || ");Serial.print(returnvect[i][j]);
  //     }
  //   }
    // Serial.print(rover.previous_roverstate);Serial.print(" || ");
    //Serial.println(rover.current_roverstate);
    //Serial.println(rover.Accelerate());
    //  Serial.print(" || ");Serial.println(String(rover.current_x));

    //counter = 0;
  

  //}

}

void FollowPattern(std::vector<std::vector<String>> instructions) {

  // instructions contains commands for TRANSLATE and ROTATE
  // these are deterministic (they will end in the IDLE state after rover has translated/rotated a certain amount)

  // instruction counter
  for (int ic = 0; ic < instructions.size(); ic ++) {
    vector 
  }

}