#include <Arduino.h>
#include "SPI.h"
#include <vector>
#include "opticalclass.cpp"

#define ROVER 2 // Rover 1 and 2 have slight differences. This allows the code to compensate.

#define PWMA 2
#define AI2 15
#define AI1 4
#define STNDBY 14
#define BI1 33
#define BI2 32
#define PWMB 25

// STATES
// 0 - IDLE
// 1 - Moving forwards/backwards (MOVE)
// 2 - Turning CW/CCW (TURN)
// 3 - Moving a distance (TRANSLATE)
// 4 - Turning an angle (ROTATE)

class DriveController {

  // private members

    // Optical sensor object
    OpticalSensor os;

    // state of the motor controller
    bool armed;

    bool translating;
    bool rotating;

    float speed; // speed to average for all the movements

    float left_speed;
    float right_speed;

    // cartesian position and orientation of the rover from 'origin'
    float x;
    float y;
    float angle;

    // 'temporary' variables to store poisition and orientation when doing something
    float current_x;
    float current_y;
    float current_angle;

    // 'temporary' variables to store initial positions and orientations before doing something
    float initial_x;
    float initial_y;
    float initial_angle;

    // variables to store desired...
    float desired_translation; // ...distance to move
    float desired_angle; // ...angle to move to

    // various gains
    float translate_kp = 1.5;
    float rotate_kp = 1;

    // sets uppeer/lower limits for saturation
    float sat_bound = 10;

    // utilities    
    float saturation(float sat_input, float upperlimit, float lowerlimit);
    

  // public members
  public:
    DriveController(OpticalSensor &os);
    void Arm();
    void Disarm();

    // set initial values
    void SetInitialValues();

    // functions to do independent wheel movement
    // speed varies from -100 to 100 here, positive moves rover forwards
    void RightWheel(float speed);
    void LeftWheel(float speed);

    // functions to do controlled actions, but indefinite amount (i.e. no setpoint angle or distance to move to)
    void Move(float speed);
    void Turn(float speed);

    void Run(bool sample);
};

float DriveController::saturation(float sat_input, float upperlimit, float lowerlimit){
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

DriveController::DriveController(OpticalSensor &optical_sensor) {
  os = optical_sensor;
}

// Triggered by interrupt to run periodically in main loop
void DriveController::Run(bool sample) {
  if (sample) {
    if (translating) {

    } else if (rotating) {

    }
  }
}

void DriveController::SetInitialValues() {
  os.UpdatePos(current_x, current_y);
  initial_x = current_x;
  initial_y = current_y;
  initial_angle = current_angle;
}

void DriveController::Move(float speed) {

  float bound = sat_bound;
  if (speed < 0) bound = -bound;

  float current_error = initial_x - current_x;

  right_speed = speed + translate_kp * current_error;
  left_speed = speed - translate_kp * current_error;

  right_speed = saturation(right_speed, speed + bound, speed - bound);
  left_speed = saturation(left_speed, speed + bound, speed - bound);

  RightWheel(right_speed);
  LeftWheel(left_speed);  

}

void DriveController::Arm(){
  digitalWrite(STNDBY, HIGH);
  armed = true;
}

void DriveController::Disarm(){
  digitalWrite(STNDBY, LOW);
  armed = false;
}

void DriveController::RightWheel(float speed){

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

