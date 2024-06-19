#include <esp_now.h> //ESP-NOW for peer to peer communication with no outside wifi network
#include <WiFi.h>    //General wifi capability
#include <esp_wifi.h>//For changing mac address

//Function call timers_______________
unsigned long lastUpdate = 0;          //Last time the main loop was run
const int frequency = 80;              //Frequency to run main loop at
const int interval = 1000 / frequency; //Interval between each code run based on frequency

//Wireless communication_____________
uint8_t macAddress[] = { 0x0C, 0xB8, 0x15, 0xC3, 0x08, 0xC0 }; //Mac address of both sender and receiver
struct data_struct{ //Struct for containing received data (Not totally necessary, but makes it easy to add other information later)
  float speed;
};
data_struct data;   //Actual variable for containing received data
esp_now_peer_info_t peerInfo;


//Misc values________________________
const bool debugEnabled  = true; //No text is printed if debug is disabled (aka false)
int interruptPin = 25;

void setup() {
  if(debugEnabled){
    Serial.begin(115200);
    while(!Serial){}
    Serial.println("Serial ready");
  }

  WiFi.mode(WIFI_STA); //Initialized wifi in station mode
  esp_wifi_set_mac(WIFI_IF_STA, macAddress); //Changes the esp's mac address (not permanently)

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

  esp_now_register_send_cb(OnDataSent); //Sets which function to be triggered when data has been sent


}

void loop() {
  if(millis() > lastUpdate + interval){ //Runs at a specific frequency as long as the esp can keep up
    lastUpdate = millis();
    
    data.speed = getSpeed();
    esp_now_send(0, (uint8_t *)&data, sizeof(data));

    

  }
}


float getSpeed(){
  float speed = 0;



  return speed;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {


}


