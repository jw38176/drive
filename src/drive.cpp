#include <Arduino.h>
#include "SPI.h"
#include <vector>
#include "drive.hpp"

#define ROVER 2 // Rover 1 and 2 have slight differences. This allows the code to compensate.

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

void update_pos(float &total_x1,float &total_y1){
  MD md;
  mousecam_read_motion(&md);
  float distance_x = convTwosComp(md.dx);
  float distance_y = convTwosComp(md.dy);
  total_x1 = total_x1 + distance_x*0.206375;
  total_y1 = total_y1 + distance_y*0.206375;
}

// saturation revised
float saturation(float sat_input, float upperlimit, float lowerlimit){
  if (sat_input > 0) {
    if (sat_input > upperlimit) sat_input = upperlimit;
    else if (sat_input < lowerlimit) sat_input = lowerlimit;
    else sat_input = sat_input;
  }  else {
    if (sat_input < upperlimit) sat_input = upperlimit;     // upperlimit is more negative than lowerlimit ==> more speed
    else if (sat_input > lowerlimit) sat_input = lowerlimit;
    else sat_input = sat_input;
  }
  return sat_input;
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

void Right(float speed){
  if ((100 > speed) && (speed > 0)) {
    RightForward(speed);
  } else if ((0 > speed) && (speed > -100)) {
    RightBackward(-speed);
  } else if (speed == 0){
    RightStop();
  } else {
  }
}

void Left(float speed){
  if ((100 > speed) && (speed > 0)) {
    LeftForward(speed);
  } else if ((0 > speed) && (speed > -100)) {
    LeftBackward(-speed);
  } else if (speed == 0){
    LeftStop();
  } else {
  }
}

// Move forwards/backwards - this just maintains the course using a P controller
// Positive speed = forwards, negative speed = backwards
void Move(float &leftspeed,float &rightspeed,float speed,float initial_x, float current_x, float kp) {
  float bound = 10;

  if (speed < 0) bound = -bound;

  float current_error = initial_x - current_x;

  rightspeed = speed + kp * current_error;
  leftspeed = speed - kp * current_error;

  rightspeed = saturation(rightspeed, speed + bound, speed - bound);
  leftspeed = saturation(leftspeed, speed + bound, speed - bound);

  Right(rightspeed);
  Left(leftspeed);  
}

// Turn on the spot - this just keeps the translational error low using a P controller. 
// Positive speed = CCW rotation, negative speed = CW rotation

void Turn(float &leftspeed,float &rightspeed,float speed,float initial_y, float current_y, float kp) {
  float bound = 10.0;
  float error_y = current_y - initial_y;

  // correct for y-translational error
  float y_corr = error_y * kp;
  leftspeed = - speed - y_corr;
  rightspeed = speed - y_corr;

  Right(rightspeed);
  Left(leftspeed);
}

// Move forwards/backwards a certain distance while maintaining course - speed can be negative
void Translate(float &leftspeed,float &rightspeed,float speed,float initial_x, float current_x, float initial_y, float current_y, float distance_to_move, float kp, bool &done_translation){
  float bound = 10.0;

  if (speed < 0) bound = -bound;

  float current_error = initial_x - current_x;

  // simple on/off controller for distance - will make proportional
  if (abs(current_y - initial_y) <= abs(distance_to_move)) {
    rightspeed = speed + kp * current_error;
    leftspeed = speed - kp * current_error;

    rightspeed = saturation(rightspeed, speed + bound, speed - bound);
    leftspeed = saturation(leftspeed, speed + bound, speed - bound);
    
    done_translation = false;

  } else {
    leftspeed = 0;
    rightspeed = 0;
    done_translation = true;
  }

  Right(rightspeed);
  Left(leftspeed);  
}

// Rotate on the spot a certain angle, trying to maintain zero translation
// Angles denoting anticlockwise rotation are positive
void Rotate(float &leftspeed,float &rightspeed,float speed,float initial_x, float current_x, float initial_y, float current_y, float &current_angle, float start_angle, float desired_angle, float kp, bool &done_rotation) {

  float bound = 10.0;
  float track_to_sensor = 128;

  if (ROVER == 2) track_to_sensor = 135;
  
  float error_y = current_y - initial_y;
  float rotate_speed = speed;

  current_angle = start_angle + ((current_x - initial_x) / track_to_sensor) * 180.0/3.14159;
  float error_angle = desired_angle - current_angle;

  // correct for angular error
  // bang-bang controller for angular error
  if (error_angle < -1) {
    leftspeed = speed;
    rightspeed = -speed;
    done_rotation = false;
  } else if (error_angle > 1) {
    leftspeed = -speed;
    rightspeed = speed;
    done_rotation = false;
  } else {
    done_rotation = true;
  }

  // correct for y-translational error
  float y_corr = error_y * 1;
  leftspeed = leftspeed - y_corr;
  rightspeed = rightspeed - y_corr;

  if (done_rotation) {
    leftspeed = 0;
    rightspeed = 0;
  }

  Right(rightspeed);
  Left(leftspeed);
}
