#include <DynamixelSDK.h>

#define ADDR_AX_ID                    3
#define ADDR_AX_TORQUE_ENABLE         24    // Torque control address
#define ADDR_AX_GOAL_POSITION         30    // Goal position address
#define ADDR_AX_PRESENT_POSITION      36    // Current position address
#define ADDR_AX_MOVING_SPEED          32    // Moving speed address

#define PROTOCOL_VERSION              1.0
#define DXL_ID                        1     // Default Dynamixel ID
#define BAUDRATE                      1000000
#define DEVICENAME                    "1"

#define TORQUE_ENABLE                 1
#define TORQUE_DISABLE                0
#define FULL_SPEED                    1023  // Full speed range
#define HALF_SPEED                    512   // Half speed range

#define OSCILLATION_CENTER            500
#define OSCILLATION_AMPLITUDE         600
#define OSCILLATION_FREQUENCY         0.5   // Hz

#include <math.h>

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

int goalPosition = OSCILLATION_CENTER;
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;
int16_t dxl_present_position = 0;
bool isHalfSpeedSet = false;               // Flag to toggle speed
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  while(!Serial.available());
  String a;
  if(Serial.available()) {
    a = Serial.readString();
    Serial.println(a); 
    int dxl_new_id = a.toInt();
    delay(2000);
  }

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (portHandler->openPort()) {
    Serial.println("Succeeded to open the port!");
  } else {
    Serial.println("Failed to open the port!");
    return;
  }

  if (portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Succeeded to change the baudrate!");
  } else {
    Serial.println("Failed to change the baudrate!");
    return;
  }

  // Enable torque and set initial speed to full range
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, FULL_SPEED, &dxl_error);

  startTime = millis();  // Track start time for oscillation
}

void loop() {
  unsigned long currentTime = millis();
  float timeInSeconds = (currentTime - startTime) / 1000.0;

  // Set the goal position to oscillate in a sinusoidal wave
  goalPosition = OSCILLATION_CENTER + OSCILLATION_AMPLITUDE * sin(2 * PI * OSCILLATION_FREQUENCY * timeInSeconds);

  // Write the goal position to the Dynamixel
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
  
  // Check if it's time to switch to half speed
  if (!isHalfSpeedSet && timeInSeconds > 10) {
    packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, HALF_SPEED, &dxl_error);
    isHalfSpeedSet = true;
    Serial.println("Moving speed set to half range.");
  }

  // Read and display the current position
  packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);

  Serial.print("Goal Position: ");
  Serial.print(goalPosition);
  Serial.print("\t Present Position: ");
  Serial.print(dxl_present_position);
  Serial.println();

  delay(20);  // Delay to limit Serial output rate
}

