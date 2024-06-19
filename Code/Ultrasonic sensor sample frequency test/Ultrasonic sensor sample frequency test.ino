
//Struct with two pins for ultrasonic sensor: trig and echo
struct ultraPin {
  int trig;
  int echo;
};

const ultraPin ultraPinL = { .trig = 26, .echo = 27 };  //Left ultrasonic sensor
const ultraPin ultraPinR = { .trig = 32, .echo = 33 };  //Right ultrasonic sensor

unsigned long lastPrint = 0;
unsigned long printInterval = 1000;
int measurements = 0;

const int sampleFrequency = 80;
int lastSample = 0;
int sampleInterval = 0; //1000 / sampleFrequency;

float distance = 0;

float averageMeasurement = 0;

void setup() {
  //Ultrasonic sensors
  pinMode(ultraPinL.trig, OUTPUT);
  pinMode(ultraPinL.echo, INPUT);

  pinMode(ultraPinR.trig, OUTPUT);
  pinMode(ultraPinR.echo, INPUT);

  Serial.begin(115200);      // Starts the serial communication

  while (!Serial) {}

  lastPrint = millis();
}

void loop() {

  if (millis() >= lastPrint + printInterval) {
    Serial.print("Samples per second: ");
    Serial.println(measurements / (printInterval / 1000));

    Serial.print("Average measurement: ");
    if(measurements == 0){
      Serial.println(0);
    }
    else{
      Serial.println(averageMeasurement / measurements);
    }
    
    measurements = 0;
    averageMeasurement = 0;
    
    lastPrint = millis();
    lastSample = millis(); //Make sure that the measurement count is accurate
  }

  if (millis() >= lastSample + sampleInterval) {
    lastSample = millis();

    //Serial.print("Distance: ");
    //Serial.println(distance);

    averageMeasurement += measureDist(ultraPinL) / 2;
    averageMeasurement += measureDist(ultraPinR) / 2;

    measurements += 1;
  }
}

float measureDist(ultraPin pin) {
  sendPulse(pin.trig);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  int duration = pulseIn(pin.echo, HIGH);
  // Calculating the distance
  float distance = duration * 0.034 / 2;

  return (distance);
}

void sendPulse(int pin) {  //Pulses a pin high for 10 microseconds
  //Clear the pin
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  // Sets the pin high for 10 microseconds
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
}

