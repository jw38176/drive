#ifndef DRIVE_H
#define DRIVE_H


#include <Arduino.h>
#include "SPI.h"
#include <vector>



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

// hw_timer_t * timer = NULL;
// volatile uint32_t isrCounter = 0;
// volatile uint32_t lastIsrAt = 0;

void ARDUINO_ISR_ATTR onTimer();
float saturation(float sat_input, float upperlimit, float lowerlimit);
float pi(float pi_input);
int convTwosComp(int b);
void mousecam_reset();
int mousecam_init();
void mousecam_write_reg(int reg, int val);
int mousecam_read_reg(int reg);

struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};

void mousecam_read_motion(struct MD *p);
int mousecam_frame_capture(byte *pdata);



void update_pos(float &total_x1, float &total_y1);
void arm();
void disarm();
void RightForward(float rightspeed);
void RightBackward(float rightspeed);
void LeftBackward(float leftspeed);
void LeftForward(float leftspeed);
void RightStop();
void LeftStop();



void Forward(float &leftspeed,float &rightspeed,float speed,float current_error);
void Backward(float speed);

void clockwise(float speed, float angle);

#endif