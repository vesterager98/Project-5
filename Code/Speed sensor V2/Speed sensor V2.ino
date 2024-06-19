unsigned long lastChange = 0;

int interruptPin = 25;

unsigned long lastPrint = 0;
int printInterval = 1000/80;

float speed = 0;

void setup() {
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt, CHANGE);

  while(!Serial){}

}

void loop() {
  if(millis() >= lastPrint + printInterval){
    Serial.print("Speed: ");
    Serial.println(speed);
  }
}

void interrupt(){
  int time = micros() - lastChange;
  lastChange = micros();

  speed = (345 / 16 / 1000) / (time / 1000000);

}
