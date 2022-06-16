#ifndef header_total_h
#define header_total_h

#include <vector>




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

  void reset();
  int conv_twos_comp(int b);
  int init();
  void write_reg(int reg, int val);
  int read_reg(int reg);
  void read_motion(struct MD *p);


  public:
    void InitSensor();
    void UpdatePos(float &x,float &y);
   
};

//extern OpticalSensor OPTICALSENSOR;

class DriveController 
{
  public:

    // Optical sensor object
    OpticalSensor os;
    // state of the motor controller
    bool first_iteration;
    float speed; // speed to average for all the movements
    float left_speed;
    float right_speed;

    float accel_counter;
    float accel_ticks = 100;

    

    // cartesian position and orientation of the rover from 'origin'
    float x;
    float y;
    float heading;
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
    float rotate_kp = 1.6;
    float rotate_y_corr_kp = 1;

    // sets uppeer/lower limits for saturation
    float sat_bound = 10;
    // utilities    
    
  // public members

        //enum RoverStates{IDLE, MOVE, TURN, TRANSLATE, ROTATE};

        //RoverStates current_roverstate = IDLE;
        //RoverStates previous_roverstate = IDLE;

        int IDLE = 0;int MOVE = 1;int TURN = 2;int TRANSLATE = 3;int ROTATE =4;
        int current_roverstate = IDLE;
        int previous_roverstate = IDLE;

        void InitController(OpticalSensor &optical_sensor);
        float saturation(float sat_input, float upperlimit, float lowerlimit);
        std::vector<std::vector<std::string>> ReturnInfo();
        float Accelerate();
        void Arm();
        void Disarm();
        void SetInitialValues();
        void RightWheel(float speed);
        void LeftWheel(float speed);
        void Move(float speed);
        void Turn(float speed);
        void Rotate(float speed);
        void Translate(float speed);
        void ChangeState(int new_roverstate);
        void SetSpeed(float set_des_speed);
        void SetAngle(float set_des_angle);
        void SetTrans(float set_des_trans);
        void Run(volatile bool &sample);
};

//extern DriveController DRIVECONTROLLER;
#endif