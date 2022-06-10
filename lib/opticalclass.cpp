#include <Arduino.h>
#include "SPI.h"
#include <vector>

#define ROVER 2 // Rover 1 and 2 have slight differences. This allows the code to compensate.

#define PIN_SS        5
#define PIN_MISO      19
#define PIN_MOSI      23
#define PIN_SCK       18

#define PIN_reset     35
#define PIN_MOUSECAM_CS        5

#define COUNTS_TO_MM_FACTOR 0.206375

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

class OpticalSensor {

  // private members
  struct MD
    {
    byte motion;
    char dx, dy;
    byte squal;
    word shutter;
    byte max_pix;
    };

  double total_x = 0;
  double total_y = 0;

  void reset();
  int conv_twos_comp(int b);
  int init();
  void write_reg(int reg, int val);
  int read_reg(int reg);
  void read_motion(struct MD *p);

  public:
    void UpdatePos(float &x,float &y);
   
};

int OpticalSensor::conv_twos_comp(int b)
{
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
}

void OpticalSensor::reset()
{
  digitalWrite(PIN_reset,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_reset,LOW);
  delay(35); // 35ms from reset to functional
}
 
int OpticalSensor::init()
{
  pinMode(PIN_reset,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  reset();
  return 1;
}

void OpticalSensor::write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int OpticalSensor::read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(1);
  return ret;
}
 
void OpticalSensor::read_motion(struct MD *p)
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

void OpticalSensor::UpdatePos(float &x,float &y){
  MD md;
  read_motion(&md);
  x = x + conv_twos_comp(md.dx) * COUNTS_TO_MM_FACTOR;
  y = y + conv_twos_comp(md.dy) * COUNTS_TO_MM_FACTOR;
}

