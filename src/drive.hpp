#include <Arduino.h>
#include "SPI.h"

//motor pins
#define PWMA 2
#define AI2 15
#define AI1 4
#define STNDBY 14
#define BI1 33
#define BI2 32
#define PWMB 25

//motor functions 
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

//motor functions with IR integration 
void AccForwardBangBang(unsigned long duration,int Speed, float &total_x, float &total_y){ 
  int rightspeed = Speed, leftspeed = Speed; 
  float error; 
  int dspeed = 10; 
  maintain_cor(total_x, total_y); 
  float init_x = total_x; 
    for(int i = 0; i < duration; i += 10){
        maintain_cor(total_x, total_y);
        error = init_x - total_x; 
        if (error < 0){ 
            rightspeed = Speed - dspeed;
            leftspeed = Speed + dspeed; 
        } else if (error > 0){ 
            rightspeed = Speed + dspeed; 
            leftspeed = Speed - dspeed; 
        } else {
            rightspeed = Speed; 
            leftspeed = Speed; 
        }
        if(rightspeed > 100) {
            rightspeed = 100; 
        }
        if (leftspeed > 100) {
            leftspeed = 100; 
        }
        RightCW(rightspeed); 
        LeftCW(leftspeed); 
        Serial.println(String(error));
    }
    RightStop();
    LeftStop();
}

void AccForward(unsigned long duration,int Speed, float &total_x, float &total_y){ 
  float rightspeed = Speed, leftspeed = Speed; 
  float error, cumerr; 
  float kp = 0.5;
  maintain_cor(total_x, total_y); 
  float init_x = total_x; 
    for(int i = 0; i < duration; i ++){
        maintain_cor(total_x, total_y); 
        error = init_x - total_x; 
        rightspeed = rightspeed - (kp * error);
        leftspeed = leftspeed + (kp * error);
        if (std::abs(error) < 0.1) {
          rightspeed = Speed; 
          leftspeed = Speed; 
        }
        RightCCW(rightspeed); 
        LeftCCW(leftspeed); 
    RightStop();
    LeftStop();
}