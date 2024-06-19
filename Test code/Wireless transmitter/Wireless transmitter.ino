#include <esp_now.h>
#include <WiFi.h>

//Wifi stuff_______________________________________
// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = { 0x0C, 0xB8, 0x15, 0xC3, 0x08, 0xC0 };

typedef struct data_struct {
  int speed;
  bool sentPulse;
} data_struct;

data_struct test;

esp_now_peer_info_t peerInfo;

//Ultrasonic stuff_________________________________
int trigPin = 23;
//int echoPin = 11; Not used on the transmitter

int pulseInterval = 1000;
unsigned long lastPulse = 0;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  
  pinMode(trigPin, OUTPUT);
  

  


  WiFi.mode(WIFI_STA);

  while(!Serial){}

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void sendPulse() {
  //Clear trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void loop() {
  test.speed = random(0, 20);
  if (millis() - lastPulse > pulseInterval) {
    lastPulse = millis();
    test.sentPulse = true;
  } else {
    test.sentPulse = false;
  }

  esp_err_t result = esp_now_send(0, (uint8_t *)&test, sizeof(data_struct));

  if (result == ESP_OK && test.sentPulse) {
    sendPulse();

  }

  delay(1000);
}