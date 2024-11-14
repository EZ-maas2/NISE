#include <DynamixelSDK.h>
#include <math.h>

// Dynamixel Control Table Addresses
#define ADDR_AX_ID                     3
#define ADDR_AX_TORQUE_LIMIT           34
#define ADDR_AX_TORQUE_ENABLE          24
#define ADDR_AX_GOAL_POSITION          30
#define ADDR_AX_PRESENT_POSITION       36

// Protocol and motor settings
#define PROTOCOL_VERSION               1.0
#define BAUDRATE                       1000000
#define DEVICENAME                     "1"
#define DXL_ID                         1
#define TORQUE_ENABLE                  1
#define TORQUE_DISABLE                 0
#define FULL_TORQUE                    1023
#define HALF_TORQUE                    512

// Neural Oscillator Parameters
#define DISCRIPTION_LENGTH             15
#define NUM_NEURONS                    20


unsigned long startTime;
unsigned long myTime;
unsigned int mydelay = 1000; // ms

int currentNeuronIndex = 0; // Start with the first neuron
unsigned long lastSwitchTime = 0; // Last time the neuron was switched
const unsigned long switchInterval = 5000; // Time in milliseconds to switch neurons (1 second)

struct Neuron {
  double x = 0.0;
  double adaptation = 0.0;
  double adaptation_tau = 12;
  double y = 0.0;
  double b = 2.5;
  double inj_cur = 0.0;
} neurons[NUM_NEURONS];

const double a =1.5;
/******************************************************/ 
// Connection matrix for mutual inhibition
struct {
  double y[NUM_NEURONS];
  double connectionMatrix[NUM_NEURONS][NUM_NEURONS] = {
    { 0, a,  0,  0,  0,  0,  0,  0, a,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 1
    { a,  0, a,  0,  0,  0,  0,  0,  0, a,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 2
    { 0, a,  0, a,  0,  0,  0,  0,  0,  0, a,  0,  0,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 3
    { 0,  0, a,  0, a,  0,  0,  0,  0,  0,  0, a,  0,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 4
    { 0,  0,  0, a,  0, a,  0,  0,  0,  0,  0,  0, a,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 5
    { 0,  0,  0,  0, a,  0, a,  0,  0,  0,  0,  0,  0, a,  0,  0, 0, 0, 0, 0 }, // Neuron 6
    { 0,  0,  0,  0,  0, a,  0, a,  0,  0,  0,  0,  0,  0, a,  0, 0, 0, 0, 0 }, // Neuron 7
    { 0,  0,  0,  0,  0,  0, a,  0, 0,  0,  0,  0,  0,  0,  0, a, 0, 0, 0, 0 }, // Neuron 8
    { a,  0,  0,  0,  0,  0,  0, 0,  0, a,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 9
    { 0, a,  0,  0,  0,  0,  0,  0, a,  0, a,  0,  0,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 10
    { 0,  0, a,  0,  0,  0,  0,  0,  0, a,  0, a,  0,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 11
    { 0,  0,  0, a,  0,  0,  0,  0,  0,  0, a,  0, a,  0,  0,  0, 0, 0, 0, 0 }, // Neuron 12
    { 0,  0,  0,  0, a,  0,  0,  0,  0,  0,  0, a,  0, a,  0,  0, 0, 0, 0, 0 }, // Neuron 13
    { 0,  0,  0,  0,  0, a,  0,  0,  0,  0,  0,  0, a,  0, a,  0, 0, 0, 0, 0 }, // Neuron 14
    { 0,  0,  0,  0,  0,  0, a,  0,  0,  0,  0,  0,  0, a,  0, a, 0, 0, 0, 0 }, // Neuron 15
    { 0,  0,  0,  0,  0,  0,  0, a,  0,  0,  0,  0,  0,  0, a,  0, 0, 0, 0, 0 }, // Neuron 16
    {1.5*a, 1.5*a, 1.5*a, 1.5*a, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0, a,  a, 0 }, // Neuron 17
    { 0,  0,  0,  0,  0,  0,  0,  0, 1.5*a, 1.5*a, 1.5*a, 1.5*a,  0,  0,  0,  0, a,  0, 0, a }, // Neuron 18
    { 0,  0,  0,  0, 1.5*a, 1.5*a, 1.5*a, 1.5*a,  0,  0,  0,  0,  0,  0,  0,  0, a, 0,  0, a }, // Neuron 19
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 1.5*a, 1.5*a, 1.5*a, 1.5*a,  0, a, a,  0 }  // Neuron 20
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

void update_network(void) {
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

  packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_LIMIT, FULL_TORQUE, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  for (int i = 0; i < NUM_NEURONS; i++) {
    neurons[i].b = 2.5;
    neurons[i].adaptation_tau = 12;
  }

  for (int i = 0; i < NUM_NEURONS; i++) {
    all_neurons.y[i] = neurons[i].y;
  }
}

void loop() {

  myTime = millis();
  
  /* Apply input current to the neurons after a delay (5000 ms) */
 
 for (int i = 0; i < 20; i++) 
 {
  if (myTime > 1000) 
  {
    neurons[i].inj_cur = 1;  // Inject current after the specified time
  } 
  else {
    neurons[i].inj_cur = 0;
  }
 }
  
  update_network();
  
  // Convert the output of the current neuron to a position within the motor's range
  goalPosition = map(neurons[0].y, 0.0, 0.7, 0, 1023); 
  Serial.println(map(0.3, 0, 0.6, 0, 1023));
  Serial.println(goalPosition);


  // Write the goal position to the motor
  packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
  
  // Read the current position of the motor
  packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
  
  // Print out the goal and present positions
  //Serial.print("Goal Position: ");
  //Serial.print(goalPosition);
  //Serial.print("\t Present Position: ");


  //Serial.println(neurons[0].y);
  
  
  delay(20); // Delay for stability
}
