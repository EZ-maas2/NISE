#include <Wire.h>
#include <ICM20948_WE.h>

#define ICM20948_ADDR 0x69 // Default I2C address, if solder jumper bridged: 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR); // Create an ICM20948 object

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(2000);
  while (!Serial);
  Serial.println("Scanning for I2C devices...");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
    }
    delay(10);
  }
  
  
  if (!myIMU.init()) {
    Serial.println("ICM20948_does_not_respond");
  } else {
    Serial.println("ICM20948_is_connected");
  }

  

  // Set Accelerometer Range
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);

  // Set Accelerometer DLPF to Level 6 for low noise
  myIMU.setAccDLPF(ICM20948_DLPF_6);

  // Set Accelerometer Sample Rate Divider
  myIMU.setAccSampleRateDivider(10);

  // Set Gyroscope Range
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);

  // Set Gyroscope DLPF to Level 6 for low noise
  myIMU.setGyrDLPF(ICM20948_DLPF_6);

  // Set Gyroscope Sample Rate Divider
  myIMU.setGyrSampleRateDivider(10);
}

void loop() {
  myIMU.readSensor();

  xyzFloat gValue = myIMU.getGValues();
  xyzFloat gyr = myIMU.getGyrValues();

  float pitch = myIMU.getPitch(); // Computed by the formula in the tutorial using arctan
  float roll = myIMU.getRoll();

  // Print Acceleration Data
  Serial.println("Acceleration_(G_values)_in_m/s^2:");
  Serial.print(gValue.x);
  Serial.print("__");
  Serial.print(gValue.y);
  Serial.print("__");
  Serial.println(gValue.z);

  // Print Pitch and Roll
  Serial.print("Pitch_=_");
  Serial.print(pitch);
  Serial.print("_Roll=_");
  Serial.println(roll);

  // Print Gyroscope Data
  Serial.println("Gyroscope_data_in_degrees/s:");
  Serial.print(gyr.x);
  Serial.print("__");
  Serial.print(gyr.y);
  Serial.print("__");
  Serial.println(gyr.z);

  Serial.println();
  Serial.println("**************");

  delay(100);
}
