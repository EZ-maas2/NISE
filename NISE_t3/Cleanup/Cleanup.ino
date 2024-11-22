#include <DynamixelSDK.h>
#include <math.h>


// ========================== NEURONS =======================
#define NUM_NEURONS                    14
unsigned int mydelay = 1000; // ms


// Array of neurons that keeps track of the state of CPG
// Doesn't have memory, identical for all neurons except x and y
struct Neuron {
  double x = 0.0;
  double adaptation = 0.0;
  double adaptation_tau = 12*0.5;
  double y = 0.0;
  double b = 2.5;
  double inj_cur = 0.0;
} neurons[NUM_NEURONS];

// --------------------- FUNCTIONS -------------------------


// =========================================================


// ========================== MOTORS ======================
// Dynamixel Control Table Addresses
#define ADDR_AX_ID                     3
#define ADDR_AX_TORQUE_LIMIT           34
#define ADDR_AX_TORQUE_ENABLE          24
#define ADDR_AX_GOAL_POSITION          30
#define ADDR_AX_PRESENT_POSITION       36
#define ADDR_AX_ANGLE_LIMIT_CW         6 //clockwise angle limit address
#define ADDR_AX_ANGLE_LIMIT_CCW        8

// Protocol and motor settings
#define PROTOCOL_VERSION               1.0
#define BAUDRATE                       1000000
#define DEVICENAME                     "1"
#define DXL_ID                         1
#define TORQUE_ENABLE                  1
#define TORQUE_DISABLE                 0
#define FULL_TORQUE                    1023
#define HALF_TORQUE                    512
#define NUM_MOTORS                     7
#define ANGLE_LIMIT_CW                768 // has to be a value between 0 and 1023 
#define ANGLE_LIMIT_CCW               256 // has to be a value between 0 and 1023 

// Array of motors that keeps track of motor state and all motor parameters
struct Motor {
  int position_limit_cw;
  int position_limit_ccw;
  int left_neuron_id;
  int right_neuron_id;
  double goalPosition = 512;
  double presentPosition;
} motors[NUM_MOTORS];

int POS_LIMITS_CW[NUM_MOTORS] =  {512 - 20, 512 - 125, 512 - 90, 512  - 125, 512 - 160, 512 -  195, 512 - 200}; // below 512
int POS_LIMITS_CCW[NUM_MOTORS] = {512 + 20, 512 + 125, 512 + 90, 512  + 125, 512 + 160, 512 +  195, 512 + 200};// above 512

// --------------------- FUNCTIONS -------------------------
void set_normal_motor_pos_limits()
{
  for(int m = 0; m < NUM_MOTORS; m++){
    motors[m].position_limit_cw = POS_LIMITS_CW[m];
    motors[m].position_limit_ccw= POS_LIMITS_CCW[m];
  }
}

// =========================================================


// ========================== SENSORS ======================
// Sensor definition
#define FL_SENSOR_PIN A0 // Front Left Flex Sensor
#define FR_SENSOR_PIN A1 // Front Right Flex Sensor
#define ML_SENSOR_PIN A2 // Rear Left Flex Sensor
#define MR_SENSOR_PIN A3 // Rear Right Flex Sensor
#define BL_SENSOR_PIN A4 
#define BR_SENSOR_PIN A5

// --------------------- FUNCTIONS -------------------------


// =========================================================

// ================= GENERAL ===============================
bool IsCurrentOn = false; 
unsigned long myTime;
long long int lastTimeCheckingSensors = 0;
// --------------------- FUNCTIONS -------------------------

// =========================================================

void setup()
{

}


void  loop()
{


}