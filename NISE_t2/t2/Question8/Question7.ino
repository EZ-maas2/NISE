#include <DynamixelSDK.h>

#define ADDR_AX_ID           3                 

#define ADDR_AX_TORQUE_ENABLE           24                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36
#define MOVING                          46

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel


// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define DXL_NEW_ID                      1                   // Dynamixel ID: 2


#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
                                                            //DEVICENAME "2" -> Serial2
                                                            //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      1000                 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define FULL_TORQUE                   1023  // Full torque range
#define HALF_TORQUE                   512   // Half torque range

#define OSCILLATION_CENTER            500
#define OSCILLATION_AMPLITUDE         300
#define OSCILLATION_FREQUENCY         0.5   // Hz

#include <math.h>
// Create PortHandler instance
dynamixel::PortHandler *portHandler;

// Create PacketHandler instance
dynamixel::PacketHandler *packetHandler;

//***********Set Global Variables****************
int goalPosition = OSCILLATION_CENTER;
int isMoving = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
uint8_t dxl_error = 0;                          // Dynamixel error
int16_t dxl_present_position = 0;               // Present position

bool isHalfTorqueSet = false;               // Flag to toggle torque
unsigned long startTime;
uint8_t dxl_new_id = DXL_NEW_ID; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial.available());
String a;
  if(Serial.available())
  {
    a= Serial.readString();// read the incoming data as string
    Serial.println(a); 
    dxl_new_id = a.toInt();
    delay(2000);
  }

  // Initialize portHandler. Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize packetHandler. Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler= dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }
  // Set initial torque to full range
  packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_LIMIT, FULL_TORQUE, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  
  
  / Change ID
  for(int id=0;id<=253;id++)
  {
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_ID, dxl_new_id, &dxl_error);
  delay(10);
  }

  startTime = millis();  // Track start time for oscillation
}

void loop() {
  unsigned long currentTime = millis();
  float timeInSeconds = (currentTime - startTime) / 1000.0;

  // Set the goal position to oscillate in a sinusoidal wave
  goalPosition = OSCILLATION_CENTER + OSCILLATION_AMPLITUDE * sin(2 * PI * OSCILLATION_FREQUENCY * timeInSeconds);

  // Write the goal position to the Dynamixel
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
  
  // Check if it's time to switch to half torque
  if (!isHalfTorqueSet && timeInSeconds > 10) {
    packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_LIMIT, HALF_TORQUE, &dxl_error);
    isHalfTorqueSet = true;
    Serial.println("Torque limit set to half range.");
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
