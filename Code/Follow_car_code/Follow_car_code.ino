//Struct with two pins for ultrasonic sensor: trig and echo
typedef struct ultraPin {
  int trig;
  int echo;
} ultraPin;

const ultraPin ultraPinL = { .trig = 10, .echo = 11 };  //Left ultrasonic sensor (Trig = Yellow, Echo = Orange)
const ultraPin ultraPinR = { .trig = 18, .echo = 19 };  //Right ultrasonic sensor


//variables for ultrasonic sensor
long duration;
int distance;

int measurements[500];
int measureIndex = 0;

//Speed controller
int motorPin = 3;     //PWM output pin
int motorSpeed = 25;  //Speed of motor

//General variables and such
bool debug = true;
unsigned long lastTime = 0;
int timeDelay = 30;

int buttonPin = 2;
int buttonPrimed = false;

bool active = true;

// H-bridge pins

int INA = 4;

//Speed increase
int speedIncreaseDelay = 3000;
int measuring = false;



void setup() {
  Serial.begin(9600);
  ///Initiate pins for ultrasonic sensors
  pinMode(ultraPinL.trig, OUTPUT);
  pinMode(ultraPinL.echo, INPUT);

  pinMode(ultraPinR.trig, OUTPUT);
  pinMode(ultraPinR.echo, INPUT);

  pinMode(buttonPin, INPUT);

  //Motor pins
  pinMode(motorPin, OUTPUT);

  //H-bridge pins
  pinMode(INA, OUTPUT);
  digitalWrite(INA, HIGH);

  while (!Serial) {
  }

  delay(1000);

  speedIncreaseDelay += millis();  //Make sure the right time passes without taking
}

void loop() {
  //Serial.print("Motor power: "); Serial.println(motorSpeed);
  if (digitalRead(buttonPin) == HIGH) {
    if (buttonPrimed) {
      Serial.println("\n\nMeasurements:\n");
      for (int i = 0; i < sizeof(measurements) / sizeof(measurements[0]); i++) {
        if (measurements[i] > 0) {
          Serial.println(measurements[i]);
        }
      }
      buttonPrimed = false;
    }
  } else {
    buttonPrimed = true;
  }



  if (speedIncreaseDelay < millis()) {
    measuring = true;
    motorSpeed = 30;

  } else if (speedIncreaseDelay - millis() <= 1000) {
    measuring = true;
  }



  if (millis() - lastTime >= timeDelay and active and measuring) {
    lastTime = millis();

    //int distL = measureDist(ultraPinL);
    //int distR = measureDist(ultraPinR);

    //int avDist = (distL + distR) / 2;
    int avDist = measureDist(ultraPinL);

    Serial.print("Distance: ");
    Serial.println(avDist);

    measurements[measureIndex] = avDist;
    measureIndex++;
    if (measureIndex >= sizeof(measurements) / sizeof(measurements[0])) {
      active = false;
    }

    if (distance <= 10) {
      active = false;
    }
  }

  if (!active) {
    //Serial.println("Motor turned off.");
    motorSpeed = 0;
  }


  analogWrite(motorPin, motorSpeed);
}


int measureDist(ultraPin pin) {
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
  int oldDistance = distance;
  distance = duration * 0.034 / 2;
  if (distance > 100) {
    distance = oldDistance;
  }
  // Prints the distance on the Serial Monitor
  /*
  if (debug) {
    Serial.print("Distance: ");
    Serial.println(distance);
  }
  */
  delay(5);

  return (distance);
}