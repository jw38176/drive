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

  double total_x = 0;
  double total_y = 0;

  void reset();
  int conv_twos_comp(int b);
  int init();
  void write_reg(int reg, int val);
  int read_reg(int reg);
  void read_motion(struct MD *p);

  public:
    OpticalSensor();
    void UpdatePos(float &x,float &y);
   
};

//extern OpticalSensor OPTICALSENSOR;

class DriveController 
{
  // private members

    // Optical sensor object
    OpticalSensor os;
    // state of the motor controller
    bool first_iteration;
    float speed; // speed to average for all the movements
    float left_speed;
    float right_speed;

    float accel_counter;
    float ticks = 100;

    

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
    float rotate_kp = 1;
    // sets uppeer/lower limits for saturation
    float sat_bound = 10;
    // utilities    


    
    enum class RoverStates{IDLE, MOVE, TURN, TRANSLATE, ROTATE};
    RoverStates current_roverstate;
    
  // public members
    public:
        DriveController(OpticalSensor &optical_sensor);
        float saturation(float sat_input, float upperlimit, float lowerlimit);
        std::vector<std::vector<std::string>> ReturnInfo(float &heading, float &x, float &y);
        float Accelerate(float accel_counter,float speed,float ticks);
        void Arm();
        void Disarm();
        void SetInitialValues();
        void RightWheel(float speed);
        void LeftWheel(float speed);
        void Move(float speed);
        void Turn(float speed);
        void Rotate(float speed,float desired_angle);
        void Translate(float speed,float desired_translation);
        void Run(float sample);
};

//extern DriveController DRIVECONTROLLER;
#endif