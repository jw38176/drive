#include <Arduino.h>
#include "SPI.h"
#include <vector>
#include "drive.hpp"



#define PWMA 2
#define AI2 15
#define AI1 4
#define STNDBY 14
#define BI1 33
#define BI2 32
#define PWMB 25
#define PIN_SS        5
#define PIN_MISO      19
#define PIN_MOSI      23
#define PIN_SCK       18

#define PIN_MOUSECAM_RESET     35
#define PIN_MOUSECAM_CS        5

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60
 
#define ADNS3080_PRODUCT_ID_VAL        0x17
#define LOW_FRAME_LOWER        0x7e
#define LOW_FRAME_UPPER        0x0e



// void ARDUINO_ISR_ATTR onTimer(){
//   bool sample = true;
// }

//saturation code
float saturation(float sat_input, float upperlimit, float lowerlimit){
  if (sat_input > upperlimit){
    sat_input = upperlimit;
  }
  else if (sat_input < lowerlimit){
    sat_input = lowerlimit;
  }
  return sat_input;
}
// pi control, not sure, not used atm
// float pi(float pi_input){
//   float e_integration;
//   e_integration = pi_input;

//   if(initial_speed >= 100){
//     e_integration = 0;
//   }
//   else if (initial_speed <= 0){
//     e_integration = 0;
//   }
  
//   delta_speed = kp*(current_error - previous_error) + ki*e_integration; //sampling time missing in ki multiplication
//   new_speed = initial_speed + delta_speed;

//   saturation(new_speed,speed_max,speed_min);

//   initial_speed = new_speed;
//   previous_error =  current_error;
  
//   return new_speed;
// }

int convTwosComp(int b)
{
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
}

void mousecam_reset()
{
  digitalWrite(PIN_MOUSECAM_RESET,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET,LOW);
  delay(35); // 35ms from reset to functional
}
 
int mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  mousecam_reset();
  return 1;
}

void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(1);
  return ret;
}
 
void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(5);
}
 
// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(byte *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_PIXEL_BURST);
  delayMicroseconds(50);
  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if(started==0)
    {
      if(pix&0x40)
        started = 1;
      else
      {
        timeout++;
        if(timeout==100)
        {
          ret = -1;
          break;
        }
      }
    }
    if(started==1)
    {
      pdata[count++] = (pix & 0x3f)<<2; // scale to normal grayscale byte range
    }
  }
 
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(14);
  return ret;
}

void update_pos(float &total_x1,float &total_y1){
  MD md;
  mousecam_read_motion(&md);
  float distance_x = convTwosComp(md.dx);
  float distance_y = convTwosComp(md.dy);
  total_x1 = total_x1 + distance_x*0.206375;
  total_y1 = total_y1 + distance_y*0.206375;
  //delay(10);//check later1
}

void arm(){
  digitalWrite(STNDBY, HIGH);
}
void disarm(){
  digitalWrite(STNDBY, LOW);
}
void RightForward(float rightspeed){
  int DutyRef = map(rightspeed,0,100,0,256);
  digitalWrite(AI1, HIGH);
  digitalWrite(AI2, LOW);
  analogWrite(PWMA , DutyRef);
}
void RightBackward(float rightspeed){
  int DutyRef = map(rightspeed,0,100,0,256);
  digitalWrite(AI1, LOW);
  digitalWrite(AI2, HIGH);
  analogWrite(PWMA , DutyRef);
} 
void LeftBackward(float leftspeed){
  int DutyRef = map(leftspeed,0,100,0,256);
  digitalWrite(BI1, LOW);
  digitalWrite(BI2, HIGH);
  analogWrite(PWMB , DutyRef);
}
void LeftForward(float leftspeed){
  int DutyRef = map(leftspeed,0,100,0,256);
  digitalWrite(BI1, HIGH);
  digitalWrite(BI2, LOW);
  analogWrite(PWMB , DutyRef);
}
void RightStop(){
  digitalWrite(AI1, LOW);
  digitalWrite(AI2, LOW);
}
void LeftStop(){
  digitalWrite(BI1, LOW);
  digitalWrite(BI2, LOW);
}
// can convert all of these to angle requirements instead of total_x
//need to add position and velocity control after making sure these work

void Forward(float &leftspeed,float &rightspeed,float speed,float current_error){
  float bound = 10.0;
  rightspeed = rightspeed + 1*current_error;
  leftspeed = leftspeed - 1*current_error;
  float rightspeed_sat = saturation(rightspeed, speed + bound,speed - bound);
  float leftspeed_sat = saturation(leftspeed, speed + bound, speed - bound );
  RightForward(rightspeed_sat);
  LeftForward(leftspeed_sat);
}
// void Backward(float speed){
//   float rightspeed = speed; float leftspeed = speed;
//   float bound;
//   update_pos();
//   float current_error = update_pos()[0];
//   float new_rightspeed = rightspeed + kp*current_error;//might need to change these
//   float new_leftspeed = leftspeed - kp*current_error;
//   float rightspeed_sat = saturation(new_rightspeed, speed + bound,speed - bound);
//   float leftspeed_sat = saturation(new_leftspeed, speed + bound, speed - bound );
//   RightBackward(rightspeed_sat);
//   LeftBackward(leftspeed_sat);
// }

// void clockwise(float speed, float angle){
//   float r = 135;
//   float angle_rad = angle * (3.14159265359/180);
//   float delta_x = angle_rad * r;
// if (update_pos()[0] < delta_x){
//     LeftForward(speed);
//     RightForward(speed);
// }
// else{
//     LeftStop();
//     RightStop();
// }
// }
