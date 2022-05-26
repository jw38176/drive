#include <Arduino.h>
#include "SPI.h"
#include <thread> 
#include <vector> 

//motor pins
#define PWMA 2
#define AI2 15
#define AI1 4
#define STNDBY 14
#define BI1 33
#define BI2 32
#define PWMB 25

//IR camera pins
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


float total_x = 0;
float total_y = 0;

float total_x1 = 0;
float total_y1 = 0;


float x=0;
float y=0;

float a=0;
float b=0;

float distance_x=0;
float distance_y=0;

volatile byte movementflag=0;
volatile int xydat[2];

int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }


int tdistance = 0;


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

struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};


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

void setup() {
    //motor initialize 
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

    Serial.begin(115200);

    if(mousecam_init()==-1)
    {
        Serial.println("Mouse cam failed to init");
        while(1);
    }

    // Tanmay changed this
    mousecam_write_reg(ADNS3080_FRAME_PERIOD_LOWER, LOW_FRAME_LOWER);
    mousecam_write_reg(ADNS3080_FRAME_PERIOD_UPPER, LOW_FRAME_UPPER);
}

std::vector<float> coordinate(float dx, float dy){
  total_x1 = total_x1 + distance_x;
  total_y1 = total_y1 + distance_y;
  total_x = total_x1/157.0;
  total_y = total_y1/157.0;
  return {total_x, total_y}; 
}

void arm(){
  digitalWrite(STNDBY, HIGH);
}
void disarm(){
  digitalWrite(STNDBY, LOW);
}
void RightCW(int Speed){
  int DutyRef = map(Speed,0,100,0,256);
  digitalWrite(AI1, LOW);
  digitalWrite(AI2, HIGH);
  analogWrite(PWMA , DutyRef);
}
void RightCCW(int Speed){
  int DutyRef = map(Speed,0,100,0,256);
  digitalWrite(AI1, HIGH);
  digitalWrite(AI2, LOW);
  analogWrite(PWMA , DutyRef);
}
void LeftCW(int Speed){
  int DutyRef = map(Speed,0,100,0,256);
  digitalWrite(BI1, LOW);
  digitalWrite(BI2, HIGH);
  analogWrite(PWMB , DutyRef);
}
void LeftCCW(int Speed){
  int DutyRef = map(Speed,0,100,0,256);
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
void RightTurn_Spot(unsigned long duration, int Speed){
  RightCCW(Speed);
  LeftCW(Speed);
  delay(duration);
  RightStop();
  LeftStop();
}
void LeftTurn_Spot(unsigned long duration,int Speed){
    LeftCCW(Speed);
    RightCW(Speed);
    delay(duration);
    RightStop();
    LeftStop();
}
void Forward(int duration,int Speed){
  LeftCW(Speed);
  RightCW(Speed);
  delay(duration);
  RightStop();
  LeftStop();
}
void Backward(unsigned long duration,int Speed){
  LeftCCW(Speed);
  RightCCW(Speed);
  delay(duration);
  RightStop();
  LeftStop();
}

void AccForward(unsigned long duration,int Speed){ 
  int rightspeed, leftspeed; 
  float error; 
  int kp = 15; 
  MD md;
  mousecam_read_motion(&md);
  std::vector<float> initial_coordinate = coordinate(md.dx, md.dy); 
    for(int i = 0; i < duration; i += 10){
        //calculate current difference 
        mousecam_read_motion(&md);
        error = initial_coordinate[0] - convTwosComp(md.dx);
        Serial.println(String(distance_x));
        if (distance_x > 0){ 
            rightspeed = Speed + kp * error; 
            leftspeed = Speed; 
        } else if (distance_x < 0){ 
            rightspeed = Speed; 
            leftspeed = Speed + kp * error; 
        } else {
            rightspeed = Speed; 
            leftspeed = Speed; 
        }
        RightCW(rightspeed); 
        LeftCW(leftspeed); 
        Serial.println(String(rightspeed) + "," + String(leftspeed));
    }
    RightStop();
    LeftStop();
}

void loop() {
  // put your main code here, to run repeatedly:
  arm();
  AccForward(4000, 60);
  disarm();

  delay(2000);
}

