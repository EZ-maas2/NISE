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
#define ANGLE_LIMIT_LOW                768 // has to be a value between 0 and 1023 
#define ANGLE_LIMIT_HIGH               256 // has to be a value between 0 and 1023 

// Neural Oscillator Parameter
#define NUM_NEURONS                    14

bool IsCurrentOn = false;


unsigned long startTime;
unsigned long myTime;
unsigned int mydelay = 1000; // ms
int motor_ix = 0;
long long int lastTime = 0;

int currentNeuronIndex = 0; // Start with the first neuron
unsigned long lastSwitchTime = 0; // Last time the neuron was switched
const unsigned long switchInterval = 5000; // Time in milliseconds to switch neurons (1 second)
const double SLOW_TAU = 50;
const double FAST_TAU = 6;


struct Motor {
  int position_limit_cw;
  int position_limit_ccw;
  int left_neuron_id;
  int right_neuron_id;
  double goalPosition = 512;
  double presentPosition;
} motors[NUM_MOTORS];



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

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    int motor_ix = i + 1;
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_TORQUE_LIMIT, FULL_TORQUE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, motor_ix, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_ANGLE_LIMIT_CW, ANGLE_LIMIT_HIGH, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, motor_ix, ADDR_AX_ANGLE_LIMIT_CCW, ANGLE_LIMIT_LOW, &dxl_error);


  }

  
}


void loop() {

  myTime = millis();
  


  
  

  // For each motor:
  // 1)Retrieve two neurons responsible for motor's movement
  // 2) Check which one is active
  // 3) Map its value to motor position
  // 4) Send this motor position to the motor

  int POS[NUM_MOTORS] = {512, 512, 512, 512, 512, 512, 512};
  for (int i = 0; i < NUM_MOTORS; i++)
    {
      motors[i].goalPosition = POS[i];
      int present_position = 0;

        // Write the goal position to the motor
      packetHandler->write2ByteTxRx(portHandler, i+1, ADDR_AX_GOAL_POSITION, motors[i].goalPosition, &dxl_error);
      packetHandler->read2ByteTxRx(portHandler, i+1, ADDR_AX_PRESENT_POSITION, (uint16_t*)&present_position, &dxl_error);
      motors[i].presentPosition = present_position;
      //Serial.print(motors[i].presentPosition);
      //Serial.print(", ");
    }
    //Serial.print(motors[0].presentPosition);
    //Serial.print("\n");



  delay(20); // Delay for stability
}
