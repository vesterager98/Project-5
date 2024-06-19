#include <esp_now.h>   //ESP-NOW for peer to peer communication with no outside wifi network
#include <WiFi.h>      //General wifi capability
#include <esp_wifi.h>  //For changing mac address

//Sensor stuff_______________________
//Struct with two pins for ultrasonic sensor: trig and echo
struct ultraPin {
  int trig;
  int echo;
};

const ultraPin ultraPinL = { .trig = 26, .echo = 27 };  //Left ultrasonic sensor
const ultraPin ultraPinR = { .trig = 32, .echo = 33 };  //Right ultrasonic sensor


//Function call timers_______________
unsigned long lastUpdate = 0;           //Last time the main loop was run
const int frequency = 80;               //Frequency to run main loop at
const int interval = 1000 / frequency;  //Interval between each code run based on frequency


//Misc values________________________
struct motorPins_struct {
  int PWMPin;        //Pin for controlling the Voltage
  int forwardsPin;   //Pin for toggling forwards direction
  int backwardsPin;  //Pin for toggling backwards direction
};
const motorPins_struct motorPins = { .PWMPin = 16, .forwardsPin = 17, .backwardsPin = 18 };
const motorPins_struct servoPins = { .PWMPin = 22, .forwardsPin = 19, .backwardsPin = 21 };
const int servoAnglePin = 34;  //Pin for measuring the angle of the servo



const bool debugEnabled = false;  //No text is printed if debug is disabled (aka false)

int leadingCarSpeed = 0;  //Speed of the leading car. Must be a global variable since it arrives at unknown times

//Wireless communication_____________
uint8_t macAddress[] = { 0x0C, 0xB8, 0x15, 0xC3, 0x08, 0xC0 };  //Mac address of both sender and receiver
struct data_struct {                                            //Struct for containing received data (Not totally necessary, but makes it easy to add other information later)
  float speed;
};
data_struct data;  //Actual variable for containing received data

bool dead = false;



void setup() {
  if (debugEnabled) {
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("Serial ready");
  }

  WiFi.mode(WIFI_STA);  //Initialized wifi in station mode

  esp_wifi_set_mac(WIFI_IF_STA, macAddress);  //Changes the esp's mac address (not permanently)

  if (esp_now_init() != ESP_OK) {
    
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);  //Sets

  pinSetup();  //Sets pins to input/output
}

void loop() {
if(dead){
  Serial.print("\n\n\n Angle exceeded \n\n\n");
}
  while (dead) {
    motorControl(motorPins, 0);
    motorControl(servoPins, 0);
    delay(1000);
  }
  /*
  while(true){
    float pep = getServoAngle();
    delay(500);
  }
  */

  if (millis() >= lastUpdate + interval) {  //Runs at a specific frequency as long as the esp can keep up
    lastUpdate = millis();

    float leftDistance = measureDist(ultraPinL);
    float rightDistance = measureDist(ultraPinR);
    motorControl(
      motorPins,
      motorController(
       (leftDistance + rightDistance) / 2,
        leadingCarSpeed));
    //motorControl(motorPins, 0);
    
    motorControl(
      servoPins,
      angleController(
        calculateAngle(leftDistance, rightDistance)));
    
    if (debugEnabled) {

      /*
      static int count = 0;
      
      Serial.print(calculateAngle(leftDistance, rightDistance));
      Serial.print(",");

      count++;

      if (count >= 25) {
        Serial.println("");
        count = 0;
      }

      
      static int counts = 0;

      Serial.print((leftDistance + rightDistance) / 2);
      Serial.print(",");

      counts++;

      if (counts >= 25) {
        counts = 0;
        Serial.println("");
      }

      */
    }

    if (millis() > lastUpdate + interval) {
      Serial.println("Timed out!");
      //dead = true;
    }
  }
  
}


int angleController(float carAngle) {

  // Definitions

  static float AngleError[] = { 0 , 0 };
  static float ServoVoltage[] = { 0 , 0 };
  static float angle_ref = 0;


  // Inputs in degrees

  float servo_ang = getServoAngle();    // Measured servo angle
  //angle_ref = -filter(carAngle);        // Reference to be followed by the servo
  angle_ref = carAngle;
  // Voltage Calculation 

  AngleError[1] = AngleError[0];
  AngleError[0] = -servo_ang + angle_ref; // The 0.72 translate from servo to wheel angle.
  ServoVoltage[1] = ServoVoltage[0];
  ServoVoltage[0] = (13.23 * AngleError[0] -12.04 * AngleError[1] + 0.1677 * ServoVoltage[1])/10;

  
  // Voltage limiter 

  if (ServoVoltage[0] > 5) {
    ServoVoltage[0] = 5;
  }

  if (ServoVoltage[0] < -5) {
    ServoVoltage[0] = -5;
  }


  // Debug

  if (debugEnabled) {
    
    Serial.print("\nError: ");
    Serial.println(AngleError[0]);
    Serial.print("\nVoltage: ");
    Serial.println(ServoVoltage[0]);
   
    
  }


  // Servo angle limiter
  if(abs(servo_ang) > 30){
    dead = true;
    return (0);
  }

  if (servo_ang > 30 && ServoVoltage[0] > 0) {
    return (0);
  } else if (servo_ang < -30 && ServoVoltage[0] < 0) {
    return (0);
  }


  // Output voltage in PWM e [0,255]

  return (int(ServoVoltage[0] / 5 * 255));
  //return(0);
}
/*
float filter(float value) { 
  static float ang_input[] = { 0, 0 };
  static float ang_output[] = { 0, 0, 0 };

  ang_input[1] = ang_input[0];
  ang_input[0] = value;
  ang_output[2] = ang_output[1];
  ang_output[1] = ang_output[0];
  ang_output[0] = 0.02773 * ang_input[0] + 0.02464 * ang_input[1] + 1.65 * ang_output[1] - 0.7022 * ang_output[2];

  return (ang_output[0]);
}
*/
float getServoAngle() {
  int raw = analogRead(servoAnglePin);
  float angle = 217.83 - raw*0.075808;

  if (angle > 30 || angle < -30) {
    dead = true;
  }

  if (debugEnabled) {
    
    Serial.print("\nServo angle: ");
    Serial.println(angle);
    
    Serial.print("Raw angle: ");
    Serial.println(raw);
    
  }
  


  return (angle);
}

int motorController(float distance, float leadingSpeed) {  //Not to be confused with motorControl. This calculates the Voltage that should be applied
  //                    | Distance from the following car to the leading car
  //                                    | Speed of the leading car

  static float distance_ref = 0.05; // 10 cm
  distance /= 100;  //Change from cm to m


  if (distance > 0.6) {
    dead = true;
    return (0);
  }

  static float distanceMemory[] = { 0, 0, 0 };
  static float VoltageMemory[] = { 0, 0, 0 };

  distanceMemory[2] = distanceMemory[1];
  distanceMemory[1] = distanceMemory[0];
  distanceMemory[0] = distance - distance_ref;
  VoltageMemory[2] = VoltageMemory[1];
  VoltageMemory[1] = VoltageMemory[0];
  VoltageMemory[0] = 2*(39.87 * distanceMemory[0] - 78.59 * distanceMemory[1] + 38.73 * distanceMemory[2]);
  VoltageMemory[0] += 1.835 * VoltageMemory[1] - 0.8353 * VoltageMemory[2];


  if (VoltageMemory[0] > 7.4) { VoltageMemory[0] = 7.4 ;}
  if (VoltageMemory[0] < -7.4) { VoltageMemory[0] = -7.4 ;}


  if (debugEnabled) {
    
    Serial.print("\nVoltage: ");
    Serial.println(VoltageMemory[0]);
    Serial.print("Distance: ");
    Serial.println(distanceMemory[0]);
    Serial.print("Distance mem: ");
    Serial.print(distanceMemory[1]);
    Serial.print(", ");
    Serial.println(distanceMemory[2]);
    Serial.print("Voltage mem: ");
    Serial.print(VoltageMemory[1]);
    Serial.print(", ");
    Serial.println(VoltageMemory[2]);
    
  }

  //Change from voltage to PWM value
  //float Voltage = (VoltageMemory[0] + feedForward(leadingSpeed));
  float Voltage = VoltageMemory[0];
  if(abs(Voltage) > 7.4){
    Voltage = Voltage/abs(Voltage) * 7.4;
  }
  
  Voltage *= 255 / 7.4;

  return int(Voltage);
}

float feedForward(float leadingSpeed) {
  static float FeedFoward_in[] = { 0, 0 };
  static float FeedFoward_out[] = { 0, 0 };

  FeedFoward_in[1] = FeedFoward_in[0];
  FeedFoward_in[0] = leadingSpeed;  //measured vel

  FeedFoward_out[1] = FeedFoward_out[0];
  FeedFoward_out[0] = 4.257 * FeedFoward_in[0] - 4.174 * FeedFoward_in[1] + 0.8365 * FeedFoward_out[1];

  if(FeedFoward_out[0] > 7.4){
    FeedFoward_out[0] = 7.4;
  }
  if(FeedFoward_out[0] < -7.4){
    FeedFoward_out[0] = -7.4;
  }

  // This controller also acts in the translatoric motor, therefore the voltage output is no longer Vcontroller[0] but a new variable v_motor:
  //v_motor = FeedFoward_out[0] +  //Voltage caused by Vcontroller
  //Note: FeedFoward_out[0], vcontroller[0] and v_motor[0] must all be limited to a max value of 5.
  return(FeedFoward_out[0]);
}

void motorControl(motorPins_struct pins, int Voltage) {  //Takes int as pwm value for motor control (negative values result in the same Voltage, but opposite direction)
  if (Voltage < 0) {                                     //Negative value means the torque is applied in the backwards direction
    digitalWrite(pins.forwardsPin, LOW);
    digitalWrite(pins.backwardsPin, HIGH);
    Voltage *= -1;  //Reverses the Voltage so we don't assign a negative pwm value (can't do that)
  } else {
    digitalWrite(pins.forwardsPin, HIGH);
    digitalWrite(pins.backwardsPin, LOW);
  }
  if (debugEnabled) {
    /*
    Serial.print("Voltage in motor control: ");
    Serial.println(Voltage);
    */
  }
  analogWrite(pins.PWMPin, Voltage);  //Applies the pwm signal to the control pin
  //analogWrite(pins.PWMPin, 0);
}


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));  //Copies the received bytes to data variable

  leadingCarSpeed = data.speed;
}


void pinSetup() {  //Sets pins to input/output
  //Ultrasonic sensors
  pinMode(ultraPinL.trig, OUTPUT);
  pinMode(ultraPinL.echo, INPUT);

  pinMode(ultraPinR.trig, OUTPUT);
  pinMode(ultraPinR.echo, INPUT);

  //Motor pins
  pinMode(motorPins.PWMPin, OUTPUT);
  pinMode(motorPins.forwardsPin, OUTPUT);
  pinMode(motorPins.backwardsPin, OUTPUT);

  //Servo pins
  pinMode(servoPins.PWMPin, OUTPUT);
  pinMode(servoPins.forwardsPin, OUTPUT);
  pinMode(servoPins.backwardsPin, OUTPUT);
  pinMode(servoAnglePin, INPUT);
}

float calculateAngle(float leftDist, float rightDist) {
  //                          | Distance measured by left sensor
  //                                            | Distance measured by right sensor
  //Calculation illustration: https://imgur.com/a/gMsM0YL

  const float sensorDist = 10.5;

  float angle = 0;

  float difference = leftDist - rightDist;

  angle = atan(difference / sensorDist);

  angle /= 2 * PI;

  angle *= -360;

  if(debugEnabled){
    Serial.print("Angle difference: ");
    Serial.print(angle);
    Serial.print("\n\n\n"); 
  }
  return angle;
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
