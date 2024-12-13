#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  Serial.println();

  WiFi.mode(WIFI_STA); // Initialize WiFi in Station mode
  Serial.print("ESP Board MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  Serial.println(WiFi.macAddress());
  delay(1000);
}
