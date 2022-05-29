#include <Arduino.h>
#include "SPI.h"
#include "drive.hpp"



bool forward = false; bool backward = false;
bool rotate = false; bool stop = false; 

hw_timer_t * timer = NULL;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

bool sample = false;

void ARDUINO_ISR_ATTR onTimer(){
  sample = true;
}

void setup() {
  Serial.begin(9600);
  noInterrupts();
  //code to interrupt every 10ms
  timer = timerBegin(0,80,true);
  timerAttachInterrupt(timer,&onTimer,true);
  timerAlarmWrite(timer,10000,true);
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
  // Tanmay changed this
  mousecam_write_reg(ADNS3080_FRAME_PERIOD_LOWER, LOW_FRAME_LOWER);
  mousecam_write_reg(ADNS3080_FRAME_PERIOD_UPPER, LOW_FRAME_UPPER);

  arm();

  interrupts();
}
void loop() {
  // put your main code here, to run repeatedly:
  // unsigned long currenttime = millis();
  // if(currenttime - previoustime >= 1){
  //   std::vector<float> speed = Setspeed(60, total_x, total_y, init_x);
  //   RightCCW(speed[0]);
  //   LeftCCW(speed[1]);
  //}
  forward = true;
  rotate = false;
  if (forward){
    if (sample){//need to test
      update_pos();
      Forward(60);

      sample = false;
    }
   }

}