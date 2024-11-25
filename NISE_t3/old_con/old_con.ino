#include <DynamixelSDK.h>
#include <math.h>
#include <algorithm>


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

const double a = 1.0;



// MAIN VERSION
struct {
  double y[NUM_NEURONS];
  double connectionMatrix[NUM_NEURONS][NUM_NEURONS] = {
    { 0, a,  0,  0,  0,  0,  0,  a, 0,  0,  0,  0,  0,  0 }, // Neuron 1
    { a,  0, a,  0,  0,  0,  0,  0,  a, 0,  0,  0,  0,  0 }, // Neuron 2
    { 0, a,  0, a,  0,  0,  0,  0,  0,  a, 0,  0,  0,  0 }, // Neuron 3
    { 0,  0, a,  0, a,  0,  0,  0,  0,  0,  a, 0,  0,  0 }, // Neuron 4
    { 0,  0,  0, a,  0, a,  0,  0,  0,  0,  0,  a, 0,  0 }, // Neuron 5
    { 0,  0,  0,  0, a,  0, a,  0,  0,  0,  0,  0,  a, 0 }, // Neuron 6
    { 0,  0,  0,  0,  0, a,  0, 0,  0,  0,  0,  0,  0,  a }, // Neuron 7
    { a,  0,  0,  0,  0,  0, 0,  0, a,  0,  0,  0,  0,  0 }, // Neuron 8
    { 0,  a,  0,  0,  0,  0,  0, a, 0, a,  0,  0,  0,  0 }, // Neuron 9
    { 0, 0,  a,  0,  0,  0,  0,  0, a,  0, a,  0,  0,  0 }, // Neuron 10
    { 0,  0, 0,  a,  0,  0,  0,  0,  0, a,  0, a,  0,  0 }, // Neuron 11
    { 0,  0,  0, 0,  a,  0,  0,  0,  0,  0, a,  0, a,  0 }, // Neuron 12
    { 0,  0,  0,  0, 0,  a,  0,  0,  0,  0,  0, a,  0, a }, // Neuron 13
    { 0,  0,  0,  0,  0, 0,  a,  0,  0,  0,  0,  0, a,  0 } // Neuron 14
  };
} all_neurons;

// --------------------- FUNCTIONS -------------------------
// Neuron functions
double relu(double x_i) { return std::max(0.0, x_i); }

double fun_dx(double x_i, const int synaptic_strength[], const double y[], int len_y, double s_i, double b, double adaptation_i) {
  double sum_y_scaled = 0.0;
  for (int j = 0; j < len_y; j++) {
    sum_y_scaled += y[j] * synaptic_strength[j];
  }
  return -x_i - sum_y_scaled + s_i - b * adaptation_i;
}

double fun_adaptation(double a_i, double adaptation_tau, double y_i) {
  return (1.0 / adaptation_tau) * (y_i - a_i);
}


void update_one_neuron(struct Neuron* neuron_pointer, int neuron_index) {
  int NUM_UPDATES = 10; // 10 
  double step = ((double)mydelay / 1000) / NUM_UPDATES;

  double x_i[NUM_UPDATES], adaptation_i[NUM_UPDATES];
  double k1, k2, k3, k4, k, l1, l2, l3, l4, l;
  
  x_i[0] = neuron_pointer->x;
  adaptation_i[0] = neuron_pointer->adaptation;

  int synaptic_s[NUM_NEURONS];
  for (int j = 0; j < NUM_NEURONS; j++) {
    synaptic_s[j] = all_neurons.connectionMatrix[neuron_index][j];
  }

  for (int i = 0; i < NUM_UPDATES - 1; i++) {
    k1 = step * fun_dx(x_i[i], synaptic_s, all_neurons.y, NUM_NEURONS, neuron_pointer->inj_cur, neuron_pointer->b, adaptation_i[i]);
    l1 = step * fun_adaptation(adaptation_i[i], neuron_pointer->adaptation_tau, neuron_pointer->y);
    
    k2 = step * fun_dx(x_i[i] + k1 / 2, synaptic_s, all_neurons.y, NUM_NEURONS, neuron_pointer->inj_cur, neuron_pointer->b, adaptation_i[i] + l1 / 2);
    l2 = step * fun_adaptation(adaptation_i[i] + l1 / 2, neuron_pointer->adaptation_tau, neuron_pointer->y);
    
    k3 = step * fun_dx(x_i[i] + k2 / 2, synaptic_s, all_neurons.y, NUM_NEURONS, neuron_pointer->inj_cur, neuron_pointer->b, adaptation_i[i] + l2 / 2);
    l3 = step * fun_adaptation(adaptation_i[i] + l2 / 2, neuron_pointer->adaptation_tau, neuron_pointer->y);
    
    k4 = step * fun_dx(x_i[i] + k3, synaptic_s, all_neurons.y, NUM_NEURONS, neuron_pointer->inj_cur, neuron_pointer->b, adaptation_i[i] + l3);
    l4 = step * fun_adaptation(adaptation_i[i] + l3, neuron_pointer->adaptation_tau, neuron_pointer->y);
    
    k = 1 / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);
    l = 1 / 6.0 * (l1 + 2 * l2 + 2 * l3 + l4);
    
    x_i[i + 1] = x_i[i] + k;
    adaptation_i[i + 1] = adaptation_i[i] + l;
  }

  neuron_pointer->x = x_i[NUM_UPDATES - 1];
  neuron_pointer->adaptation = adaptation_i[NUM_UPDATES - 1];
  neuron_pointer->y = relu(neuron_pointer->x);
  all_neurons.y[neuron_index] = neuron_pointer->y;
}

void update_network(void) {

  for (int i = 0; i < NUM_NEURONS; i++) {
    //update_one_neuron(&neurons[i], i);
    update_one_neuron(&neurons[i], i);
  }
}


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
#define ADDR_AX_MOVING_SPEED          32    // Moving speed address

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
#define ANGLE_LIMIT_CW                256 // has to be a value between 0 and 1023 
#define ANGLE_LIMIT_CCW               768 // has to be a value between 0 and 1023 
#define FULL_SPEED                    1023  // Full speed range


// Array of motors that keeps track of motor state and all motor parameters
struct Motor {
  int position_limit_cw;
  int position_limit_ccw;
  int left_neuron_id;
  int right_neuron_id;
  double goalPosition = 512;
  double presentPosition;
} motors[NUM_MOTORS];

// int POS_LIMITS_CW[NUM_MOTORS] =  {512 + 20, 512 + 125, 512 + 90, 512  + 125, 512 + 160, 512 +  195, 512 + 200}; 
// int POS_LIMITS_CCW[NUM_MOTORS] = {512 - 20, 512 - 125, 512 - 90, 512  - 125, 512 - 160, 512 -  195, 512 - 200};

int POS_LIMITS_CW[NUM_MOTORS] =  {512 + 12, 512 + 75, 512 + 54, 512  + 75, 512 + 96, 512 +  117, 512 + 120}; 
int POS_LIMITS_CCW[NUM_MOTORS] = {512 - 12, 512 - 75, 512 - 54, 512  - 75, 512 - 96, 512 -  117, 512 - 120};


// Dynamixel SDK Objects
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

uint8_t dxl_error = 0;
int16_t dxl_present_position = 0;
int goalPosition = 512;  // Initialize midpoint position

// --------------------- FUNCTIONS -------------------------
void set_normal_motor_pos_limits()
{
  for(int m = 0; m < NUM_MOTORS; m++){
    motors[m].position_limit_cw = POS_LIMITS_CW[m];
    motors[m].position_limit_ccw= POS_LIMITS_CCW[m];
  }
}



// custom mapping function to support floats
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {

  x = std::min(0.5f, x); 
  return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
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

void check_sensors()
{
  double FL_SENSOR_PIN_THR = 150; // THRESHOLDS AFTER WHICH WE TAKE SOME ACTION
  double FR_SENSOR_PIN_THR = 190;
  double ML_SENSOR_PIN_THR = 75;
  double MR_SENSOR_PIN_THR = 30;
  double BL_SENSOR_PIN_THR = 250;
  double BR_SENSOR_PIN_THR = 250;

  int sensor0 = analogRead(FL_SENSOR_PIN);
  int sensor1 = analogRead(FR_SENSOR_PIN);
  int sensor2 = analogRead(ML_SENSOR_PIN);


  if (sensor0 < FL_SENSOR_PIN_THR) 
  { // OBSTACLE ON THE LEFT
    Serial.println("Obstacle on the left");
    // Change pos limit
     for (int m = 0; m < NUM_MOTORS; m++)
     {motors[m].position_limit_cw = 1023;}

     //reduce_cpg_amplitudes_left();
  }

  else if (sensor1 < FR_SENSOR_PIN_THR) 
  { // OBSTACLE ON  THE RIGHT
    Serial.println("Obstacle on the right");
    for (int m = 0; m < NUM_MOTORS; m++)
    // Change pos limit
    {motors[m].position_limit_ccw = 0;}
    //reduce_cpg_amplitudes_right();
  }

  else if (sensor0 >= FL_SENSOR_PIN_THR && sensor1 >= FR_SENSOR_PIN_THR)
  {
    set_normal_motor_pos_limits();
  }
}



// ------------------ Functions --------------------------
void reduce_cpg_amplitudes_left()
{
  // if right  sensor is triggered, we reduce amplitudes of all the right neurons
  for (int ix = 0; ix < NUM_NEURONS; ix++){
    if(ix > 7)
    {
      neurons[ix].y = neurons[ix].y * 0.5;
    }
  }
}

void reduce_cpg_amplitudes_right(){
  // if left  sensor is triggered, we reduce amplitudes of all the left neurons

  for (int ix = 0; ix < NUM_NEURONS; ix++){
    if(ix < 8)
    {
      neurons[ix].y = neurons[ix].y * 0.5;
    }
  }}


void reduce_cpg_all_amplitudes(float factor)
{
  for (int ix = 0; ix < NUM_NEURONS; ix++)
  {
  neurons[ix].y = neurons[ix].y * factor;  
  }
}


void change_adaptation_tau(double new_tau){
  for (int ix = 0; ix < NUM_NEURONS; ix++)
  {
  neurons[ix].adaptation_tau = new_tau;
  }
}

// =========================================================

// ================= GENERAL ===============================
bool IsCurrentOn = false; 
unsigned long myTime;
long long int lastTimeCheckingSensors = 0;
// --------------------- FUNCTIONS -------------------------

// =========================================================

void setup()
{
  
  Serial.begin(115200);
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (portHandler->openPort() && portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Port opened and baudrate set.");
  } else {
    Serial.println("Failed to open port.");
    return;
  }

  
  // SETUP ALL MOTORS

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    int motor_ix = i + 1;
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_TORQUE_LIMIT, FULL_TORQUE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, motor_ix, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, FULL_SPEED, &dxl_error);
    
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_ANGLE_LIMIT_CW, ANGLE_LIMIT_CW, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_ANGLE_LIMIT_CCW, ANGLE_LIMIT_CCW, &dxl_error);

    motors[i].left_neuron_id = i;
    motors[i].right_neuron_id = i + 7; // such that for motor 1 corresponding neurons are 0 and 7, etc  

    // NEW STUFF
    motors[i].position_limit_cw = POS_LIMITS_CW[i];
    motors[i].position_limit_ccw = POS_LIMITS_CCW[i];

  }

  for (int i = 0; i < NUM_NEURONS; i++) {
    all_neurons.y[i] = neurons[i].y;
  }

}


void  loop()
{

myTime = millis();

int indexes[NUM_NEURONS] = {0, 7, 1, 8, 2, 9, 3, 10, 4, 11, 5, 12, 6, 13};

// Give current to all neurons
 if (myTime >  750 && !IsCurrentOn)
 {
  for (int i = 0; i < NUM_NEURONS; i++) 
  {
    neurons[indexes[i]].inj_cur = 1.0;
  }
   IsCurrentOn = true; // ensures that we only do this once
 }

// Check Sensors every 
  if (myTime > lastTimeCheckingSensors + 500)
  {
    check_sensors();
    lastTimeCheckingSensors = millis();
  }



  // CPG neurons update
  update_network();

  double sum_left = 0;
  double sum_right = 0;
  for (int i = 0; i < 7; i++)
  {
    if (i<8){
      sum_left +=  neurons[i].y;
    }
    else{
      sum_right += neurons[i].y;
    }
    
    Serial.print(neurons[i].y + i);
    Serial.print(", ");
    Serial.print(neurons[i+7].y + i);
    Serial.print(", ");

  }
  // Serial.print(sum_left);
  // Serial.print(", ");
  // Serial.print(sum_right);
  Serial.println();
  
  // Convert neuron output to the motor position

  // For each motor:
  // 1)Retrieve two neurons responsible for motor's movement
  // 2) Check which one is active
  // 3) Map its value to motor position
  // 4) Send this motor position to the motor



  if (myTime < 2000){
    for (int i =0; i < NUM_MOTORS; i++){

       int present_position = 0;

        // Write the goal position to the motor
      packetHandler->write2ByteTxRx(portHandler, i+1, ADDR_AX_GOAL_POSITION, 512, &dxl_error);
      packetHandler->read2ByteTxRx(portHandler, i+1, ADDR_AX_PRESENT_POSITION, (uint16_t*)&present_position, &dxl_error);
      motors[i].presentPosition = present_position;

    }
  }

  if (myTime > 2000){
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      // motor_ix = i + 1;
      Neuron left =  neurons[motors[i].left_neuron_id]; // retrieve the left neuron of this motor
      Neuron right = neurons[motors[i].right_neuron_id];

      if (left.y > right.y)
      {
        // 0.35 was the best
        motors[i].goalPosition = mapFloat(left.y, 0.0, 0.5, 512, motors[i].position_limit_ccw); // 512 is a neutral position, 0 is leftmost position
        //motors[i].goalPosition += bias[i];
      }

      else if (right.y > left.y)
      {
        motors[i].goalPosition = mapFloat(right.y, 0.0, 0.5, 512, motors[i].position_limit_cw); // 512 is a neutral position, 1023 is rightmost position
      }

      else 
      {
        motors[i].goalPosition = 512;
      }
      
      int present_position = 0;

        // Write the goal position to the motor
      packetHandler->write2ByteTxRx(portHandler, i+1, ADDR_AX_GOAL_POSITION, motors[i].goalPosition, &dxl_error);
      packetHandler->read2ByteTxRx(portHandler, i+1, ADDR_AX_PRESENT_POSITION, (uint16_t*)&present_position, &dxl_error);
      motors[i].presentPosition = present_position;

    }}

    delay(10);


}