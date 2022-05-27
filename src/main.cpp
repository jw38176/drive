#include "optical_sensor.hpp"
#include "drive.hpp"


//global variable for robot coordinates
float total_x = 0;
float total_y = 0; 
float init_x = 0;
bool clk; 
unsigned long previoustime = 0; 

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

  Serial.begin(9600);

  if(mousecam_init()==-1)
  {
      Serial.println("Mouse cam failed to init");
      while(1);
  }

  // Tanmay changed this
  mousecam_write_reg(ADNS3080_FRAME_PERIOD_LOWER, LOW_FRAME_LOWER);
  mousecam_write_reg(ADNS3080_FRAME_PERIOD_UPPER, LOW_FRAME_UPPER);

  arm();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currenttime = millis(); 
  if(currenttime - previoustime >= 1){ 
    std::vector<float> speed = Setspeed(60, total_x, total_y, init_x); 
    RightCCW(speed[0]);
    LeftCCW(speed[1]);
  }

}

