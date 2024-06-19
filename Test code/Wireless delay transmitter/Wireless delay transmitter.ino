#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

//Wifi stuff_______________________________________
// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress[] = { 0x0C, 0xB8, 0x15, 0xC3, 0x08, 0xC0 };

typedef struct data_struct {
  int speed;
} data_struct;

data_struct test;

esp_now_peer_info_t peerInfo;

//Trigger pin for instantaneous transmit
int trigPin = 23;


unsigned long sendTime = 0;

//Timing calibration
const int timingPoints = 20;
unsigned int delays[timingPoints];
int delaysIndex = 0;

unsigned int sendDelay = 1000000;

unsigned long lastTime = 0;

long timingOffset = 0;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //char macStr[18];

  if (status == ESP_NOW_SEND_SUCCESS && delaysIndex < timingPoints) {
    delays[delaysIndex] = micros() - sendTime;

    delaysIndex++;
  }

  Serial.print("Acknowlegement delay: ");
  Serial.println(micros() - sendTime);


  /*
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
*/
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  sendPulse();
  lastTime -= micros() - floor(micros() / sendDelay) * sendDelay;
}

void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);


  WiFi.mode(WIFI_STA);

  esp_wifi_set_mac(WIFI_IF_STA, broadcastAddress);

  while (!Serial) {}

  Serial.print("Mac address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  delay(2000);
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
  if (micros() < lastTime + sendDelay) {
    return;
  }

  sendPulse();

  lastTime = micros();

  test.speed = random(0, 20);

  esp_err_t result = esp_now_send(0, (uint8_t *)&test, sizeof(data_struct));
  sendTime = micros();

  if (result == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Succesful transfer");
  } else {
    Serial.println("Unsuccesful transfer");
  }

  


  
}