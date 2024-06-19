#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

//Wifi variables
int data = 0; //Data to be sent
uint8_t macAddress[] = { 0x0C, 0xB8, 0x15, 0xC3, 0x08, 0xC0 }; //Mac address of both sender and receiver

//Misc variables
int interruptPin = 23;


//Timing variables
unsigned long syncOffset = 0;
unsigned long lastPulse = 0; //Last time a test pulse was sent
unsigned long interval = 200000;
int ackTime = 178; //Average time for acknowledgement to reach the other controller (microseconds)
bool calibrating = true;

void setup() {
  Serial.begin(115200);

  pinMode(interruptPin, OUTPUT);
  digitalWrite(interruptPin, LOW);


  WiFi.mode(WIFI_STA);

  esp_wifi_set_mac(WIFI_IF_STA, macAddress);

  while (!Serial) {}

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  delay(200);

  while(calibrating){}

}

void loop() {
  if(syncedMicros() < lastPulse + interval){ //Doesn't run the loop unless the right amount of time has passed
    return;
  }

  lastPulse = syncedMicros();

  sendPulse();


}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  delayMicroseconds(ackTime);
  
  syncOffset = micros();

  lastPulse = 0;
  Serial.println("Synchronised");
  calibrating = false;
}

unsigned long syncedMicros(){
  return(micros() - syncOffset);
}



void sendPulse() {

  //Clear trigger pin
  digitalWrite(interruptPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(interruptPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(interruptPin, LOW);

  Serial.println("Pulse sent");
}
