//Distance/ultrasonic

//Struct with two pins for ultrasonic sensor: trig and echo
struct ultraPin {
  int trig;
  int echo;
};

const struct ultraPin ultraPinL = { .trig =, .echo = };  //Left ultrasonic sensor
const struct ultraPin ultraPinR = { .trig =, .echo = };  //Right ultrasonic sensor

struct distMem {  //Struct for saving distance measurements and the time they were made
  unsigned long time;
  int dist;
};

struct distMem distances[10];  //Cyclic distance memory to calculate speed
int distIndex = 0;

//Speed


//Angle
const float sensorDist = 5;  //Distance between ultrasonic sensors
float currentAngle = 0;

//Motor control
int forwPin = ;
int backPin = ;
int speedPin = ;

//FreeRTOS
TaskHandle_t task_measure;
TaskHandle_t task_motorControl;
TaskHandle_t task_servoControl;

//General
bool debug = true;

void setup() {
  Serial.begin(115200);
  pinSetup(); //Initiate pins for I/O





  while (!Serial) {}

  freeRTOSSetup(); //Initiate freeRTOS tasks

}

void pinSetup() {
  //Initiate pins for ultrasonic sensors
  pinMode(ultraPinL.trig, OUTPUT);
  pinMode(ultraPinL.echo, INPUT);

  pinMode(ultraPinR.trig, OUTPUT);
  pinMode(ultraPinR.echo, INPUT);

  //Initiate pins for motor control
  pinMode(forwPin, OUTPUT);
  pinMode(backPin, OUTPUT);
  pinMode(speedPin, OUTPUT);

}

void freeRTOSSetup() {
  xTaskCreate(measure, "measure", 10000, NULL, 1, &task_measure);
  delay(500);  //Check if this needs to be here
  xTaskCreate(motorControl, "motorControl", 10000, NULL, 1, &task_motorControl);
  delay(500);  //Check if this needs to be here
  xTaskCreate(servoControl, "servoControl", 10000, NULL, 1, &task_servoControl);
  delay(500);  //Check if this needs to be here
}

void loop() {
  // put your main code here, to run repeatedly:
}

void motorControl() {

  while(true){

  }
}

void servoControl() {
  
  while(true){
    
  }
}

void measure() { //Measures distance and angle to the car in front
  while (true) {
    int distL = measureDist(ultraPinL);
    int distR = measureDist(ultraPinR);

    struct distMem distance = { .dist = (distL + distR) / 2, .time = millis() };  //Average of both sensors to get measurement in the middle inlcuding time of measurement

    saveDist(distance);  //Save measured distance to array for speed calculation

    currentAngle = angleCalc(distL, distR); //Calculate and remember angle to car in front
  }
}

void saveDist(struct distMem distance) {  //Receives a dstance measurement with the time it was made and saves it appropriately
  //                            | Distance to remember

  distances[distIndex] = distance;        //Save to cyclic memory

  distIndex++;  //Increase placement of next measurement

  if (distIndex >= sizeof(distances) / sizeof(distances[0])) {  //Ensure that memory doesn't over-index
    distIndex = 0;
  }
}

float speedCalc(){ //

  return(0.0);
}

float angleCalc(int distL, int distR) { //Returns the angle to the car in front using the distances measured by two parallel sensors and the distance between them
  //                  | Distance measured by left sensor
  //                             | Distance measured by right sensor
  //Calculation illustration: https://imgur.com/a/gMsM0YL

  float angle = 0;


  int difference = distL - distR;

  angle = asin(difference / sensorDist);

  return angle;
}

int measureDist(struct ultraPin pin) { //Returns distance measured on an unltrasonic sensor
  //                             | Pins of ultrasonic sensor to measure on (ultraPin contains two pins: echo and trig)

  //Clear trigger pin
  digitalWrite(pin.trig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(pin.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin.trig, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(pin.echo, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  if (debug) {
    Serial.print("Distance: ");
    Serial.println(distance);
  }
}