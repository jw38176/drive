#include <Arduino.h>
#include "SPI.h"
#include <Wire.h>
#include "drive.hpp"
#include <OneBitDisplay.h>


bool translate = false;
bool rotate = false;

bool stop = false; 
int counter = 0;
bool done_translate = false;
bool done_rotate = false;
float accel_count = 0;

bool first_iteration = true;

hw_timer_t * timer = NULL;

volatile bool sample = false;

void ARDUINO_ISR_ATTR onTimer(){
  sample = true;
  counter = counter + 1;
}

OBDISP obd; // OneBitDisplay library init

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

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(STNDBY, OUTPUT);
  //IR camera initialize
  pinMode(PIN_SS,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);
  pinMode(PIN_SCK,OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  if(mousecam_init()==-1)
  {
      Serial.println("Mouse cam failed to init");
      while(1);
  }

  arm();

  mousecam_reset();

  interrupts();

  rotate = false;
  translate = true;

}

// 'state' variables (should be migrated heavily to the drive.cpp library)

// instantaneous speeds of each motor
float leftspeed=0;
float rightspeed=0; 

// average speed to maintain
float speed = 40;

// total_x and total_y are cumulative distances
float total_x=0; 
float total_y=0; 

// initial_x and initial_y are useful as references
float initial_x=0;
float initial_y=0; 

// angle (cumulative)
float angle = 0;

// error variables, useful to debug
float error_currentx; 
float error_currenty;
float error_currentangle;
float display_error; // Error that gets displayed on the OLED - multipurpose

// Get the initial position values to use as reference, helpful for relative rotation and translation
void set_initial_values() {
  initial_x = total_x; initial_y = total_y;
  first_iteration = false;
  mousecam_reset();
}

// write useful state variables to the OLED display
void update_oled() {

  char szTemp[32];
  obdFill(&obd, 0x0, 1);

  sprintf(szTemp, "Err : %f", display_error);   obdWriteString(&obd, 0,0,0,(char *)szTemp, FONT_12x16, 1, 1);
  sprintf(szTemp, "Speed : %d", (int)speed);      obdWriteString(&obd, 0,0,22,(char *)szTemp, FONT_8x8, 0, 1);
  sprintf(szTemp, "Left  : %d", (int)leftspeed);  obdWriteString(&obd, 0,0,30,(char *)szTemp, FONT_8x8, 0, 1);
  sprintf(szTemp, "Right : %d", (int)rightspeed); obdWriteString(&obd, 0,0,38,(char *)szTemp, FONT_8x8, 0, 1);

  if (translate || !rotate) {
    sprintf(szTemp, "Dist  : %d", (int)(total_y - initial_y));      obdWriteString(&obd, 0,0,46,(char *)szTemp, FONT_8x8, 0, 1);
  } else if (rotate) {
    sprintf(szTemp, "Angle : %d", (int)angle);      obdWriteString(&obd, 0,0,46,(char *)szTemp, FONT_8x8, 0, 1);
  }

  if      (translate)     obdWriteString(&obd, 0,0,52,(char *)"Translating...", FONT_8x8, 0, 1);
  else if (rotate)    obdWriteString(&obd, 0,0,52,(char *)"Rotating...", FONT_8x8, 0, 1);
  else obdWriteString(&obd, 0,0,52,(char *)"Doing nothing...", FONT_8x8, 0, 1);

}

// Modulates the speed to increase linearly for the 'ticks' cycles of the counter
// Time to set speed depends on the sampling period. 1 tick = 1 cycle of the interrupt counter
// Remember to set the counter to zero when finished moving
void accelerate(float &target_speed, float &accel_counter, float ticks) {
  if (accel_counter <= ticks) {
    target_speed = (int)(target_speed * (accel_counter/ticks));
    accel_counter = accel_counter + 1;
  } else {
    target_speed = target_speed;
  }
}


void loop() {

  speed = 60;

  // Update the OLED - see function for what it displays
  if (counter >= 100) {
      update_oled();
      counter = 0;
  }

  if (sample) {

    update_pos(total_x,total_y);    
    if (first_iteration) set_initial_values();

    sample = false;
    
    // Move forward an indefinite time/distance
    if (translate) {
      accelerate(speed, accel_count, 400);  // Accelerate for 100 ticks of sample clock (200ms) BETA
      TranslatePI(leftspeed, rightspeed, speed, initial_x, total_x, initial_y, total_y, 1000, 3, done_translate);
      display_error = total_x - initial_x; // Display the error from the initial x position
    } 

    else if (rotate) {
      accelerate(speed, accel_count, 100);  // Accelerate for 100 ticks of sample clock (200ms) BETA
      Rotate(leftspeed, rightspeed, speed, initial_x, total_x, initial_y, total_y, angle, 0, 90, 1, done_rotate);
      display_error = total_y - initial_y; // Display the translational error (controller tries to make this zero for a pure rotation)
    }
    
    else {
      first_iteration = true;
      accel_count = 0;
    }

    // for Sean::
    // this is an example of how to switch between translate/rotate
    // the first iteration variable must be set to TRUE when changing states
    // the done_rotate and done_translate flags must be set to FALSE when changing states
    // this will be packaged neatly into a DriveController class (in development)
    
    if (done_translate) {
      translate = false;
      first_iteration = true;
      rotate = false;
      done_translate = true;
    } if (done_rotate) {
      rotate = false;
      first_iteration = true;
      translate = true;
      done_rotate = false;
    }
  }
}