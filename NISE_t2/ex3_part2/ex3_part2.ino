#define DISCRIPTION_LENGTH     15 // constant definition (via preprocessor directive, old C feature)
#define NUM_NEURONS 2 // Now we have only 2 neurons as per Figure 2

unsigned long int myTime;
unsigned int mydelay = 50; // ms

/******************************************************/ 
/* Mutually inhibiting neurons (Adaptation of Matsuoka 1985) */
/******************************************************/ 
struct Neuron {
  double x = 0.0;
  double adaptation = 0.0;
  double adaptation_tau = 12; 
  double y = 0.0;  // Neuron output
  double b = 2.5;  // Scaling the adaptation term
  double inj_cur = 0.0;  // External current input
} neurons[NUM_NEURONS];  // Two neurons

/******************************************************/ 
// Connection matrix for mutual inhibition
struct {
  double y[NUM_NEURONS];
  int connectionMatrix[NUM_NEURONS][NUM_NEURONS] = {{0, 2.5}, {2.5, 0}};  // Mutual inhibition
} all_neurons;

/******************************************************/ 
// Non-linear function for neuron output
double relu(double x_i) {
  return max(0.0, x_i);  // Rectified Linear Unit activation function
}

/******************************************************/ 
// Function for dx/dt (based on the equation in the figure)
double fun_dx(double x_i, const int synaptic_strength[], const double y[], int len_y, double s_i, double b, double adaptation_i) {
  double sum_y_scaled = 0.0;
  for (int j = 0; j < len_y; j++) {
    sum_y_scaled += y[j] * synaptic_strength[j];  // Summing weighted synaptic inputs
  }
  return -x_i - sum_y_scaled + s_i - b * adaptation_i;  // Equation for neuron dynamics
}

/******************************************************/ 
// Function for adaptation dynamics (da/dt)
double fun_adaptation(double a_i, double adaptation_tau, double y_i) {
  return (1.0 / adaptation_tau) * (y_i - a_i);  // Adaptation dynamics equation
}

/******************************************************/ 
/* put your setup code in setup(), to run once */
void setup() {
  Serial.begin(115200);
  
  // Initialize neuron properties
  neurons[0].b = 2.5;  // Set adaptation scaling for neuron 1
  neurons[1].b = 2.5;  // Set adaptation scaling for neuron 2
  
  neurons[0].adaptation_tau = 12;  // Time constant for neuron 1
  neurons[1].adaptation_tau = 12;  // Time constant for neuron 2
  
  for (int i = 0; i < NUM_NEURONS; i++) {
    all_neurons.y[i] = neurons[i].y;  // Initialize neuron outputs in the global struct
  }
}

/******************************************************/ 
// Update a single neuron's state using Runge-Kutta method
void update_one_neuron(struct Neuron* neuron_pointer, int neuron_index) {
  int NUM_UPDATES = 10;
  double step = ((double)mydelay / 1000) / NUM_UPDATES;  // Step size for integration
  
  double x_i[NUM_UPDATES], adaptation_i[NUM_UPDATES]; 
  double k1, k2, k3, k4, k, l1, l2, l3, l4, l;
  
  x_i[0] = neuron_pointer->x; 
  adaptation_i[0] = neuron_pointer->adaptation;

  int synaptic_s[NUM_NEURONS];  // Connection weights for the neuron
  for (int j = 0; j < NUM_NEURONS; j++) {
    synaptic_s[j] = all_neurons.connectionMatrix[neuron_index][j];  // Get relevant synaptic connections
  }

  // Runge-Kutta ODE Solver
  for (int i = 0; i < NUM_UPDATES - 1; i++) {
    k1 = step * fun_dx(x_i[i], synaptic_s, all_neurons.y, NUM_NEURONS, neuron_pointer->inj_cur, neuron_pointer->b, adaptation_i[i]);
    l1 = step * fun_adaptation(adaptation_i[i], neuron_pointer->adaptation_tau, neuron_pointer->y);
    
    k2 = step * fun_dx(x_i[i] + k1 / 2, synaptic_s, all_neurons.y, NUM_NEURONS, neuron_pointer->inj_cur, neuron_pointer->b, adaptation_i[i] + l1 / 2);
    l2 = step * fun_adaptation(adaptation_i[i] + l1 / 2, neuron_pointer->adaptation_tau, neuron_pointer->y);
    
    k3 = step * fun_dx(x_i[i] + k2 / 2, synaptic_s, all_neurons.y, NUM_NEURONS, neuron_pointer->inj_cur, neuron_pointer->b, adaptation_i[i] + l2 / 2);
    l3 = step * fun_adaptation(adaptation_i[i] + l2 / 2, neuron_pointer->adaptation_tau, neuron_pointer->y);
    
    k4 = step * fun_dx(x_i[i] + k3, synaptic_s, all_neurons.y, NUM_NEURONS, neuron_pointer->inj_cur, neuron_pointer->b, adaptation_i[i] + l3);
    l4 = step * fun_adaptation(adaptation_i[i] + l3, neuron_pointer->adaptation_tau, neuron_pointer->y);
    
    k = 1 / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);  // Final estimate for x
    l = 1 / 6.0 * (l1 + 2 * l2 + 2 * l3 + l4);  // Final estimate for adaptation
    
    x_i[i + 1] = x_i[i] + k;  // Update x
    adaptation_i[i + 1] = adaptation_i[i] + l;  // Update adaptation
  }

  neuron_pointer->x = x_i[NUM_UPDATES - 1];  // Final value for x
  neuron_pointer->adaptation = adaptation_i[NUM_UPDATES - 1];  // Final value for adaptation
  neuron_pointer->y = relu(neuron_pointer->x);  // ReLU function for output
  
  all_neurons.y[neuron_index] = neuron_pointer->y;  // Update global output
}

/******************************************************/ 
// Update the entire neural network
void update_network(void) {
  for (int i = 0; i < NUM_NEURONS; i++) {
    update_one_neuron(&neurons[i], i);
  }
}

/******************************************************/ 
/* put your main code here in loop(), to run repeatedly */
void loop() {
  /* Read program running time in milliseconds */
  myTime = millis();
  
  /* Apply input current to the neurons after a delay (5000 ms) */
 
  if (myTime > 5000) {
    neurons[0].inj_cur = 1;  // Inject current after 5 seconds
  } else {
    neurons[0].inj_cur = 0;
  }
  
  if (myTime > 6000) {
    neurons[1].inj_cur = 1;  // Inject current after 5 seconds
  } else {
    neurons[1].inj_cur = 0;
  }
  
  update_network();  // Update the state of the neural network
  
  /* Print neuron outputs */
  for (int i = 0; i < NUM_NEURONS; i++) {
    Serial.print(neurons[i].y);
    Serial.print(" ");
  }
  Serial.println();

  /* Delay at the end of each loop */
  delay(mydelay);
}