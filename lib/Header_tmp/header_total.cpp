#include <Arduino.h>
#include "SPI.h"
#include <vector>
#include <string>
#include "header_total.hpp"

#define ROVER 2 // Rover 1 and 2 have slight differences. This allows the code to compensate.

#if ROVER == 1
    float track_to_sensor = 128;
#else
    float track_to_sensor = 135;
#endif

#define PWMA    2
#define AI2     15
#define AI1     4
#define STNDBY  14
#define BI1     33
#define BI2     32
#define PWMB    25

#define PIN_SS        5
#define PIN_MISO      19
#define PIN_MOSI      23
#define PIN_SCK       18

#define PIN_reset     26
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

void OpticalSensor::InitSensor() {
    pinMode(PIN_SS,OUTPUT);
    pinMode(PIN_MISO,INPUT);
    pinMode(PIN_MOSI,OUTPUT);
    pinMode(PIN_SCK,OUTPUT);    
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);  
    if(init()==-1)
    {
      while(1);
    }
    reset();
}

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
  x = x + (float)conv_twos_comp(md.dx) * 0.206375;
  y = y + (float)conv_twos_comp(md.dy) * 0.206375;
}

void DriveController::InitController(OpticalSensor &optical_sensor){
    os = optical_sensor;

    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(AI1, OUTPUT);
    pinMode(AI2, OUTPUT);
    pinMode(BI1, OUTPUT);
    pinMode(BI2, OUTPUT);
    pinMode(STNDBY, OUTPUT);

}

float DriveController::saturation(float sat_input, float upperlimit, float lowerlimit)
{
    if (sat_input > 0) 
    {
        if (sat_input > upperlimit) sat_input = upperlimit;
        else if (sat_input < lowerlimit) sat_input = lowerlimit;
        else sat_input = sat_input;
    }  else 
    {
        if (sat_input < upperlimit) sat_input = upperlimit;     // upperlimit is more negative than lowerlimit ==> more speed
        else if (sat_input > lowerlimit) sat_input = lowerlimit;
        else sat_input = sat_input;
    }
    return sat_input;
}
//check later
std::vector<std::vector<std::string>> DriveController::ReturnInfo()
{
    std::string coordinate = ("("+std::to_string(x)+" , "+std::to_string(y)+")");
    std::string state;

    if(current_roverstate == IDLE) state = "Idle";
    if(current_roverstate == MOVE) state = "Move";
    if(current_roverstate == TURN) state = "Turn";
    if(current_roverstate == ROTATE) state = "Rotate";
    if(current_roverstate == TRANSLATE) state = "Translate";

    std::vector <std::vector <std::string>> info_vect;
    std::vector <std::string> new_info;
    new_info.push_back(coordinate);new_info.push_back(std::to_string(heading));new_info.push_back(state);
    info_vect.push_back(new_info);
    return info_vect;    
}

float DriveController::Accelerate()
{
    float new_speed = speed;
    if (accel_counter <= accel_ticks) {
        new_speed = new_speed * (accel_counter/accel_ticks);
        accel_counter = accel_counter + 1;
    } 
    return new_speed;
}

void DriveController::Arm(){
    digitalWrite(STNDBY, HIGH);
}

void DriveController::Disarm(){
    digitalWrite(STNDBY, LOW);
}

void DriveController::SetInitialValues() {
    os.UpdatePos(initial_x, initial_y);
    initial_angle = current_angle;
    //first_iteration = false;
    //os.UpdatePos(current_x,current_y);
    //initial_x = current_x;initial_y=current_y;
}

void DriveController::RightWheel(float speed)
{
// convert speed to an integer.
    int int_speed = (int)speed;
    int duty_cycle = 0;
// forward movement
    if ((100 > int_speed) && (int_speed > 0)) {
        duty_cycle = map(int_speed,0,100,0,256);
        digitalWrite(AI1, HIGH);
        digitalWrite(AI2, LOW);
    // backward movement
    } else if ((0 > int_speed) && (int_speed > -100)) {
        duty_cycle = map(-int_speed,0,100,0,256);
        digitalWrite(AI1, LOW);
        digitalWrite(AI2, HIGH);
    // either zero or out of bounds: stop
    // MUST apply saturation when using this to avoid unwanted stopping
    } else {
        digitalWrite(AI1, LOW);
        digitalWrite(AI2, LOW);
    }
    analogWrite(PWMA , duty_cycle);
}

void DriveController::LeftWheel(float speed){
    // convert speed to an integer.
    int int_speed = (int)speed;
    int duty_cycle = 0;
    // forward movement
    if ((100 > int_speed) && (int_speed > 0)) {
        duty_cycle = map(int_speed,0,100,0,256);
        digitalWrite(BI1, HIGH);
        digitalWrite(BI2, LOW);
    // backward movement
    } else if ((0 > int_speed) && (int_speed > -100)) {
        duty_cycle = map(-int_speed,0,100,0,256);
        digitalWrite(BI1, LOW);
        digitalWrite(BI2, HIGH);  
    // either zero or out of bounds: stop
    // MUST apply saturation when using this to avoid unwanted stopping
    } else {
        digitalWrite(BI1, LOW);
        digitalWrite(BI2, LOW);
    }
    analogWrite(PWMB , duty_cycle);
    }

void DriveController::Move(float speed) {
    if (speed < 0) sat_bound = -sat_bound;
    float current_error = initial_x - current_x;
    right_speed = speed + translate_kp * current_error;
    left_speed = speed - translate_kp * current_error;
    right_speed = saturation(right_speed, speed + sat_bound, speed - sat_bound);
    left_speed = saturation(left_speed, speed + sat_bound, speed - sat_bound);
    RightWheel(right_speed);
    LeftWheel(left_speed);
}

        // needs editing
void DriveController::Turn(float speed){
    float error_y = current_y - initial_y;
    float y_corr = error_y * translate_kp;
    left_speed = - speed - y_corr;
    right_speed = speed - y_corr;
    RightWheel(right_speed);
    LeftWheel(left_speed);
    }

void DriveController::Rotate(float speed)
{
    float error_y = current_y - initial_y;
    current_angle = ((current_x - initial_x) / track_to_sensor) * 180.0/3.14159;
    float error_angle = desired_angle - current_angle;
    float set_speed = error_angle * rotate_kp;
    set_speed = saturation(set_speed, speed, 20);
    // correct for y-translational error
    //might need to add kp_... instead of 1 and 1.6
    float y_corr = error_y * rotate_y_corr_kp;
    left_speed = -set_speed - y_corr;
    right_speed = set_speed - y_corr;
    if(abs(error_angle)<2) {
        left_speed = 0;
        right_speed = 0;
        ChangeState(IDLE);
    }
    
    RightWheel(right_speed);
    LeftWheel(left_speed);
    
}

void DriveController::Translate(float speed)
{
    if (speed < 0) sat_bound = -sat_bound;
    float current_error = initial_x - current_x;
    float distance_error = (current_y-initial_y) - desired_translation;
    // Proportional distance control
    float set_speed = -distance_error * 1;
    set_speed = saturation(set_speed, speed, 20);
    // Set right and left speeds to correct for x-error
    right_speed = set_speed + translate_kp * current_error;
    left_speed = set_speed - translate_kp * current_error;
    right_speed = saturation(right_speed, set_speed + sat_bound, set_speed - sat_bound);
    left_speed = saturation(left_speed, set_speed + sat_bound, set_speed - sat_bound);
    // finished
    if (abs(distance_error) > 2 && left_speed > 10 && right_speed > 10) {
        ChangeState(TRANSLATE);
    } else {
        left_speed = 0;
        right_speed = 0;
        ChangeState(IDLE);
    }
    RightWheel(right_speed);
    LeftWheel(left_speed); 
}
    // Triggered by interrupt to run periodically in main loop
void DriveController::ChangeState(int new_roverstate){
    previous_roverstate = current_roverstate;
    current_roverstate = new_roverstate;
}

void DriveController::SetSpeed(float set_des_speed)
{
    speed = set_des_speed;
}

void DriveController::SetAngle(float set_des_angle)
{
    desired_angle = set_des_angle;
}

void DriveController::SetTrans(float set_des_trans)
{
    desired_translation = set_des_trans;
}

void DriveController::Run(volatile bool &sample) 
{
    if (sample) {
        //Serial.println("SAMPLE HAPPENED");
        Arm();
        os.UpdatePos(current_x,current_y);
        
        if (previous_roverstate != current_roverstate)
        {
            SetInitialValues();
            accel_counter = 0;
            previous_roverstate = current_roverstate;

            if ((previous_roverstate == MOVE) || (previous_roverstate == TRANSLATE)){
                x += current_x - initial_x;
                y += current_y - initial_y;
                heading = heading + atan(x/y);

            }else if ((previous_roverstate == TURN) || (previous_roverstate == ROTATE)){
                heading += ((int)current_angle % 360);
            }          


            //ReturnInfo();
        }
        
        if(current_roverstate == IDLE);
        else if(current_roverstate == MOVE) Move(Accelerate());
        else if(current_roverstate == TURN) Turn(Accelerate());
        else if(current_roverstate == TRANSLATE) Translate(Accelerate());
        else if(current_roverstate == ROTATE)Rotate(Accelerate());
        sample = false;
    }
}
