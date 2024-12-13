#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR ESP RECEIVER’S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x0C,0xDC,0x7E,0xCA,0xE0,0x6C}; //first board (DOIT-esp32 V1)

typedef struct test_struct {
    int x;
    int y;
} test_struct;

test_struct test;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    char macStr[18];
    Serial.print("Packet to: ");

    // Copies the sender MAC address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
    Serial.print(" send status: \t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register the send callback
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Register first peer
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    test.x = random(0, 20);
    test.y = random(0, 20);

    esp_err_t result = esp_now_send(0, (uint8_t *)&test, sizeof(test_struct));
    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }
    delay(500);
}
