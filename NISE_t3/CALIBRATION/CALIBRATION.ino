// Number of sensors
#define NUM_SENSORS 4

// Sensor pins (adjust according to your setup)
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3};
int sensorThr[NUM_SENSORS];

void setup() {
  Serial.begin(9600);

  Serial.println("Sensor Calibration");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Calibrating Sensor ");
    Serial.println(i);

    Serial.println("Please bend the sensor and type 'done' when ready.");
    waitForUserInput();
    sensorThr[i] = analogRead(sensorPins[i]);
    Serial.print("Threshold value recorded: ");
    Serial.println(sensorThr[i]);

    Serial.println("Calibration for this sensor is complete.\n");
  }

  Serial.println("All sensors calibrated successfully!");
}

void loop() {
  // Read sensors and map values using calibrated min/max
  for (int i = 0; i < NUM_SENSORS; i++) {
    int reading = analogRead(sensorPins[i]);
    // Print calibrated values
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(reading);

    if (reading < sensorThr[i]){
      Serial.print(i);
      Serial.println(" is Bent");
    }
  }

  delay(500); // Adjust delay as needed
}

void calibrate_sensors(){
  Serial.println("Sensor Calibration");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Calibrating Sensor ");
    Serial.println(i);

    Serial.println("Please bend the sensor and type 'done' when ready.");
    wait_user_input();
    sensorThr[i] = analogRead(sensorPins[i]);
    Serial.print("Threshold value recorded: ");
    Serial.println(sensorThr[i]);

    Serial.println("Calibration for this sensor is complete.\n");
  }

  Serial.println("All sensors calibrated successfully!");
}


void  wait_user_input() {
  while (true) {
    if (Serial.available() > 0) {
      String userInput = Serial.readStringUntil('\n');
      userInput.trim(); // remove whitespace
      if (userInput.equalsIgnoreCase("done")) {
        break; 
      } else {
        Serial.println("Invalid input. Please type 'done' to continue.");
      }
    }
  }
}
