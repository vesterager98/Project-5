#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

//Wifi variables
int data = 0;                                                   //Data to be sent
uint8_t macAddress[] = { 0x0C, 0xB8, 0x15, 0xC3, 0x08, 0xC0 };  //Mac address of both sender and receiver

//Misc variables
int interruptPin = 23;
esp_now_peer_info_t peerInfo;

//Timing variables
unsigned long syncOffset = 0;
unsigned long lastPulse = 0;  //Last time a test pulse was received
const unsigned long interval = 200000;
bool calibrating = true;

unsigned long calibrationInterval = 1000000;
unsigned long lastCalibration = 0;

unsigned long printDelay = interval;
unsigned long lastPrint = 0;

long errorTime = NULL;
int newlineCount = 0;

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(interruptPin, INPUT);

  while (!Serial) {}  //Wait until serial is ready

  Serial.println("\n\nSerial ready");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  esp_wifi_set_mac(WIFI_IF_STA, macAddress);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  memcpy(peerInfo.peer_addr, macAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }


  attachInterrupt(digitalPinToInterrupt(interruptPin), trigInterrupt, RISING);

  esp_now_register_send_cb(OnDataSent);

  delay(100);

  calibrate();
}

void loop() {
  if (syncedMicros() > lastPrint + printDelay && errorTime != NULL) {
    lastPrint = syncedMicros();
    //Serial.print("Error delay: ");
    Serial.print(errorTime);
    Serial.print(",");
    newlineCount++;
    if (newlineCount > 20) {
      Serial.println("");
      newlineCount = 0;
    }
  }

  if (syncedMicros() > lastCalibration + calibrationInterval) {
    lastCalibration = syncedMicros();

    calibrate();
  }
}


void calibrate() {
  calibrating = true;
  esp_now_send(0, (uint8_t *)&data, sizeof(data));
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    syncOffset = micros();

    lastPulse = 0;  //Resets last code run

    calibrating = false;

    lastPrint = 0;

    lastCalibration = 0;

    //Serial.println("Synchronised");
  }
  else{
    Serial.println("Failed synchronisation");
    calibrate();
  }
}

unsigned long syncedMicros() {
  return (micros() - syncOffset);
}


void trigInterrupt() {  //Interrupt that is triggered when the test pin is pulled high
  lastPulse = syncedMicros();


  errorTime = syncedMicros() - floor(syncedMicros() / interval) * interval;
  //Serial.print("Error delay: "); Serial.println(time);

  if (errorTime >= interval / 2) {
    errorTime -= interval;
  }
}
