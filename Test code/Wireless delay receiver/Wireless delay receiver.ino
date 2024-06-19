#include <esp_now.h>
#include <WiFi.h>

//Structure example to receive data
//Must match the sender structure
typedef struct data_struct {
  int speed;
} data_struct;

int trigPin = 23;

//Create a struct_message called myData
data_struct Data;

unsigned long triggerTime = 0;

const unsigned long receiveDelay = 1000000; //1000000 = 1 second

//Timing calibration
const int timingPoints = 20; //How many wireless transfers should be used in calculating
long expectedOffsets[timingPoints]; //The measured offsets from the expected timing
int offsetsIndex = 0;

int actualOffset = 0;
const int responseTime = 178;

bool calibrating = true;

bool receivingAverage = true;

//Transmitting variables
uint8_t broadcastAddress[] = { 0x0C, 0xB8, 0x15, 0xC3, 0x08, 0xC0 };

esp_now_peer_info_t peerInfo;

unsigned long lastPulse = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if(status == ESP_NOW_SEND_SUCCESS){
      long delay = micros() - lastPulse;

      //Serial.print("Delay: "); 
      Serial.println(delay);

      calibrating = false;

      actualOffset = delay - responseTime;

    }

}




//callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  return;
  memcpy(&Data, incomingData, sizeof(Data));

  Serial.print("Received speed: "); Serial.println(Data.speed);


  //Serial.println("Callback triggered");
}

void trigInterrupt(){
  lastPulse = micros();

  if(!calibrating){
    int time = micros() - floor(micros() / receiveDelay) * receiveDelay - actualOffset;
    Serial.print("Error delay: "); Serial.println(time);
    
  }

}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(trigPin, INPUT);

  while (!Serial) {}
  Serial.println("\n\nSerial ready");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer

  

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }


  attachInterrupt(digitalPinToInterrupt(trigPin), trigInterrupt, RISING);

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  delay(1000);

  //Calibration
  delayMicroseconds((floor(micros() / receiveDelay) + 1) * receiveDelay - micros());
  //Serial.println(micros());
  while(calibrating){
    esp_now_send(0, (uint8_t *)&Data, sizeof(data_struct));
    //Serial.print("Result: "); Serial.println(result);
    delay(1000);

  }


}

void loop() {
  

  delay(1000);
  
}