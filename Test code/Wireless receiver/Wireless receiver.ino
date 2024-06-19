#include <esp_now.h>
#include <WiFi.h>

//Structure example to receive data
//Must match the sender structure
typedef struct data_struct {
  int speed;
  bool sentPulse;
} data_struct;

typedef struct ultraPin {
  int trig;
  int echo;
} ultraPin;

ultraPin leftSensor = { .trig = 22, .echo = 23 };
//ultraPin rightSensor = {.trig = 18, .echo = 19};

//Create a struct_message called myData
data_struct Data;

unsigned long duration = 0;
float distance = 0;

bool printPrimed = false;

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&Data, incomingData, sizeof(Data));

  if (Data.sentPulse) {
    
    //Clear trigger pin
    digitalWrite(leftSensor.trig, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(leftSensor.trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(leftSensor.trig, LOW);

    duration = pulseIn(leftSensor.echo, HIGH);

    distance = duration * 0.034;
    

    printPrimed = true;

    Serial.println("Callback triggered");

  }
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(leftSensor.trig, OUTPUT);
  pinMode(leftSensor.echo, INPUT);

  while (!Serial) {}
  Serial.println("Serial ready");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (printPrimed) {
    Serial.print("Distance: ");
    Serial.println(distance);

    Serial.print("Speed: ");
    Serial.println(Data.speed);
    Serial.println();

    printPrimed = false;
  }
  
}