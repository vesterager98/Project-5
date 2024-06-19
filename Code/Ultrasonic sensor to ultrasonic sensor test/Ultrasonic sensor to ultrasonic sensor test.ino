
int trigPin1 = 10;
//int echoPin1 = 11; //Not used
int trigPin2 = 6;  //Covered with tape or something else to block it
int echoPin2 = 7;

int duration = 0;
float distance = 0;



void setup() {
  Serial.begin(9600);

  pinMode(trigPin1, OUTPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  while(!Serial){}
}

void loop() {
  //Clear trigger pin
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin2, HIGH);

  distance = duration * 0.034;

  Serial.print("Distance: "); Serial.println(distance);


  delay(200);
}
