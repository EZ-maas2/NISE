#include <DynamixelSDK.h>
#include <math.h>

// Dynamixel Control Table Addresses
#define ADDR_AX_ID                     3
#define ADDR_AX_TORQUE_LIMIT           34
#define ADDR_AX_TORQUE_ENABLE          24
#define ADDR_AX_GOAL_POSITION          30
#define ADDR_AX_PRESENT_POSITION       36
#define ADDR_AX_ANGLE_LIMIT_CW         6 //clockwise angle limit address
#define ADDR_AX_ANGLE_LIMIT_CCW        8

// Sensor definition
#define FL_SENSOR_PIN A0 // Front Left Flex Sensor
#define FR_SENSOR_PIN A1 // Front Right Flex Sensor
#define ML_SENSOR_PIN A2 // Rear Left Flex Sensor
#define MR_SENSOR_PIN A3 // Rear Right Flex Sensor
#define BL_SENSOR_PIN A4 
#define BR_SENSOR_PIN A5

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
#define ANGLE_LIMIT_LOW                725 // has to be a value between 0 and 1023 
#define ANGLE_LIMIT_HIGH               298 // has to be a value between 0 and 1023 

// Neural Oscillator Parameters
#define DISCRIPTION_LENGTH             15
#define NUM_NEURONS                    14


unsigned long startTime;
unsigned long myTime;
unsigned int mydelay = 1000; // ms
int motor_ix = 0;

int currentNeuronIndex = 0; // Start with the first neuron
unsigned long lastSwitchTime = 0; // Last time the neuron was switched
const unsigned long switchInterval = 5000; // Time in milliseconds to switch neurons (1 second)
bool suppress_updates = false;

struct Neuron {
  double x = 0.0;
  double adaptation = 0.0;
  double adaptation_tau = 12*0.5;
  double y = 0.0;
  double b = 2.5;
  double inj_cur = 0.0;
} neurons[NUM_NEURONS];


struct Motor {
  int position_limit_cw;
  int position_limit_ccw;
  int left_neuron_id;
  int right_neuron_id;
  double goalPosition = 512;
  double presentPosition;
} motors[NUM_MOTORS];


const double a =1.0;
/******************************************************/ 
// Connection matrix for mutual inhibition
struct {
  double y[NUM_NEURONS];
  double connectionMatrix[NUM_NEURONS][NUM_NEURONS] = {
    { 0, a,  0,  0,  0,  0,  0,  a, a,  0,  0,  0,  0,  0 }, // Neuron 1
    { 0,  0, a,  0,  0,  0,  0,  a,  a, a,  0,  0,  0,  0 }, // Neuron 2
    { 0, 0,  0, a,  0,  0,  0,  0,  a,  a, a,  0,  0,  0 }, // Neuron 3
    { 0,  0, 0,  0, a,  0,  0,  0,  0,  a,  a, a,  0,  0 }, // Neuron 4
    { 0,  0,  0, 0,  0, a,  0,  0,  0,  0,  a,  a, a,  0 }, // Neuron 5
    { 0,  0,  0,  0, 0,  0, a,  0,  0,  0,  0,  a,  a, a }, // Neuron 6
    { 0,  0,  0,  0,  0, 0,  0, 0,  0,  0,  0,  0,  a,  a }, // Neuron 7
    { a,  a,  0,  0,  0,  0, 0,  0, a,  0,  0,  0,  0,  0 }, // Neuron 8
    { a,  a,  a,  0,  0,  0,  0, 0, 0, a,  0,  0,  0,  0 }, // Neuron 9
    { 0, a,  a,  a,  0,  0,  0,  0, 0,  0, a,  0,  0,  0 }, // Neuron 10
    { 0,  0, a,  a,  a,  0,  0,  0,  0, 0,  0, a,  0,  0 }, // Neuron 11
    { 0,  0,  0, a,  a,  a,  0,  0,  0,  0, 0,  0, a,  0 }, // Neuron 12
    { 0,  0,  0,  0, a,  a,  a,  0,  0,  0,  0, 0,  0, a }, // Neuron 13
    { 0,  0,  0,  0,  0, a,  a,  0,  0,  0,  0,  0, 0,  0 } // Neuron 14
  };
} all_neurons;

// Dynamixel SDK Objects
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

uint8_t dxl_error = 0;
int16_t dxl_present_position = 0;
int goalPosition = 512;  // Initialize midpoint position

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


// custom mapping function to support floats
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


void update_one_neuron(struct Neuron* neuron_pointer, int neuron_index) {
  int NUM_UPDATES = 10;
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

void update_network() {
  if (suppress_updates) {
    suppress_updates = false; // Skip this update cycle
    return;
  }

  for (int i = 0; i < NUM_NEURONS; i++) {
    update_one_neuron(&neurons[i], i);
  }
}


void setup() {
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

  // MOTOR ANGLE IS +- 75 DEGREES, POSITION LIMIT IS INDIVIDUAL
  int POS_LIMITS_CW[NUM_MOTORS] = {512 - 20, 512 - 125, 512 - 90, 512  - 125, 512 - 160, 512 -  195, 512 - 200}; // below 512
  int POS_LIMITS_CCW[NUM_MOTORS] = {512 + 20, 512 + 125, 512 + 90, 512  + 125, 512 + 160, 512 +  195, 512 + 200};// above 512
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    int motor_ix = i + 1;
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_TORQUE_LIMIT, FULL_TORQUE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, motor_ix, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_ANGLE_LIMIT_CW, ANGLE_LIMIT_HIGH, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_ANGLE_LIMIT_CCW, ANGLE_LIMIT_LOW, &dxl_error);

    motors[i].left_neuron_id = i;
    motors[i].right_neuron_id = i + 7; // such that for motor 1 corresponding neurons are 1 and 8, etc  

    // NEW STUFF
    motors[i].position_limit_cw = POS_LIMITS_CW[i];
    motors[i].position_limit_ccw = POS_LIMITS_CCW[i];


  }

  for (int i = 0; i < NUM_NEURONS; i++) {
    all_neurons.y[i] = neurons[i].y;
  }
}

void check_sensors()
{
  double FL_SENSOR_PIN_THR = 400; // THRESHOLDS AFTER WHICH WE TAKE SOME ACTION
  double FR_SENSOR_PIN_THR = 400;
  double ML_SENSOR_PIN_THR = 400;
  double MR_SENSOR_PIN_THR = 400;
  double BL_SENSOR_PIN_THR = 400;
  double BR_SENSOR_PIN_THR = 400;

  int sensor0 = analogRead(FL_SENSOR_PIN);
  int sensor1 = analogRead(FR_SENSOR_PIN);

  if (sensor0 < FL_SENSOR_PIN_THR) { // OBSTACLE ON THE LEFT
    neurons[0].y = 1.0; // if left sensor is triggered, turn head right
    reduce_cpg_amplitudes_left();
  }
  else if (sensor1 < FR_SENSOR_PIN_THR) { // OBSTACLE ON  THE RIGHT
    neurons[7].y = 1.0; // if right sensor is triggered, turn head left
    reduce_cpg_amplitudes_right();
  }
  //all_neurons.y[neuron_index] = neuron_pointer->y;
}


//bool suppress_updates = false; // Global flag for suppressing updates

void reduce_cpg_amplitudes_left() {
  suppress_updates = true; // Temporarily suppress updates
  for (int ix = 0; ix < NUM_NEURONS; ix++) {
    if (ix > 7) { // Reduce right neuron outputs
      neurons[ix].y *= 0.5;
    }
  }
}

void reduce_cpg_amplitudes_right() {
  suppress_updates = true; // Temporarily suppress updates
  for (int ix = 0; ix < NUM_NEURONS; ix++) {
    if (ix < 8) { // Reduce left neuron outputs
      neurons[ix].y *= 0.5;
    }
  }
}


void loop() {

  myTime = millis();
  
  /* Apply input current to the neurons after a delay (5000 ms) */
 
 for (int i = 0; i < NUM_NEURONS; i++) 
 {
  if (myTime > 1000) 
  {
    neurons[i].inj_cur = 1;  // Inject current after the specified time
  } 
  else {
    neurons[i].inj_cur = 0;
  }
 }
  
  check_sensors();
  update_network();

  // For each motor:
  // 1)Retrieve two neurons responsible for motor's movement
  // 2) Check which one is active
  // 3) Map its value to motor position
  // 4) Send this motor position to the motor
  for (int i = 0; i < NUM_MOTORS; i++)
    {
      // motor_ix = i + 1;
      Neuron left = neurons[motors[i].left_neuron_id]; // retrieve the left neuron of this motor
      Neuron right = neurons[motors[i].right_neuron_id];

      if (left.y > right.y)
      {
        motors[i].goalPosition = mapFloat(left.y, 0, 1, 512, motors[i].position_limit_ccw); // 512 is a neutral position, 0 is leftmost position
      }

      else 
      {
        motors[i].goalPosition = mapFloat(right.y, 0, 1, 512, motors[i].position_limit_cw); // 512 is a neutral position, 1023 is rightmost position
      }
      
      int present_position = 0;
        // Write the goal position to the motor
      packetHandler->write2ByteTxRx(portHandler, i+1, ADDR_AX_GOAL_POSITION, motors[i].goalPosition, &dxl_error);
      packetHandler->read2ByteTxRx(portHandler, i+1, ADDR_AX_PRESENT_POSITION, (uint16_t*)&present_position, &dxl_error);
      motors[i].presentPosition = present_position;
      Serial.print(motors[i].presentPosition);
      Serial.print(", ");
    }

    Serial.print("\n");



  delay(20); // Delay for stability
}
