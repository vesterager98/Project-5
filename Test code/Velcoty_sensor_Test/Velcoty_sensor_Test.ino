
const byte interruptPin = 3;

float velocity = 0;
unsigned long timeData[3];
unsigned long oldTime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(interruptPin),refletive, CHANGE);
  oldTime = micros();
  //pinMode(interruptPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(velocity);
 
  //Serial.println(digitalRead(interruptPin));
}


void refletive() {
  //Serial.println("YES");

  // Get new time
  unsigned long curTime = micros();
  
  // Get time since last data point
  timeData[0] = curTime - oldTime;

  // Update old time
  oldTime = curTime;

  // Calc velocity
  if ((((timeData[0] + timeData[1] + timeData[2]) / 3) * pow(10, -3)) != 0) {
  velocity = (21.5 * pow(10.0, -3.0)) / (((timeData[0] + timeData[1] + timeData[2]) / 3.0) * pow(10, -6.0));
  }
  // Make space in array
  timeData[2] = timeData[1];
  timeData[1] = timeData[0];
  
}

void refletivee(){
  Serial.println("I see!");
}
