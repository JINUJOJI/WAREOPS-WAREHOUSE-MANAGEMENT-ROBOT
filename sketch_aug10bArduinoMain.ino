#include <avr/wdt.h> // Watchdog timer library for Arduino reset functionality

// ------------------- SPEED SETTINGS -------------------
// MOTOR_SPEED: Normal movement speed
// pivot_speed: Speed used for turning on the spot
// adjust: Reduced speed for small adjustments
#define MOTOR_SPEED 130
#define pivot_speed 165
#define adjust 120

// ------------------- ULTRASONIC SENSOR PINS -------------------
// First ultrasonic sensor (used for lift height measurement)
#define TRIG_PIN1 14
#define ECHO_PIN1 15
// Second ultrasonic sensor (used for object detection in front)
#define TRIG_PIN 19 
#define ECHO_PIN 18 

// ------------------- DRIVE MOTOR CONTROL PINS -------------------
// Motor A control pins
int in1 = 8;
int in2 = 9;
// Motor B control pins
int in3 = 6;
int in4 = 7;
// PWM pins for motor speed control
int ena = 10;
int enb = 5;

// ------------------- LIFT MOTOR CONTROL PINS -------------------
// lenb: PWM for lift motor
// lin4, lin3: Direction control for lift motor
int lenb = 11;
int lin4 = 12;
int lin3 = 13;

// ------------------- LINE SENSOR PINS -------------------
// left and right IR line sensors
int left = 3;
int right = 2;

// ------------------- SENSOR STATES -------------------
int LeftState = 1;
int RightState = 1;

// ------------------- TASK COUNTERS -------------------
// counter: Tracks overall progress at intersections
// rkcounter: Tracks position inside rack navigation
// crkcounter: Tracks position inside cross rack navigation
int counter = 0 ;
int rkcounter = 0 ;
int crkcounter = 0 ;

// ------------------- SETUP FUNCTION -------------------
// Initializes pins, sensors, motors, and serial communication
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  // Line sensors with pull-up enabled
  pinMode(left, INPUT);
  pinMode(right, INPUT);
  digitalWrite(left, HIGH);
  digitalWrite(right, HIGH);

  // Lift motor pins
  pinMode(lenb,OUTPUT);
  pinMode(lin4,OUTPUT);
  pinMode(lin3,OUTPUT);
 
  // Ultrasonic sensors
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);

  // Lift motor initial speed
  analogWrite(lenb,100);
}

// ------------------- MAIN LOOP -------------------
// Runs line following until intersection is detected
// At intersection, calls modules based on counter value
void loop() {
  while(true){
    if(digitalRead(left) == 0 && digitalRead(right) == 0){
      forward();
    }
    else if (digitalRead(left) == 0 && digitalRead(right) == 1){
      slright();
    }
    else if(digitalRead(left) == 1 && digitalRead(right) == 0){
      slleft();
    }
    else if(digitalRead(left) == 1 && digitalRead(right) == 1){
      counter = counter + 1;
      stop();
      delay(1000);
      if (counter <=3 || counter == 6){
        IntersectionModule();
      }
      if(counter == 4){
        sortingRK();
        delay(1000);
      }
      if(counter == 7){
        sortingCRK();
        delay(1000);
      }
    }
  }
}

// ------------------- PIVOT FUNCTIONS -------------------
// Rotates robot in place until line is detected
void pivotleft(){
  analogWrite(ena,adjust);
  analogWrite(enb,pivot_speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(300);
  while(digitalRead(left)==0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}

void pivotright(){
  analogWrite(ena,pivot_speed);
  analogWrite(enb,adjust);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(300);
  while(digitalRead(right)==0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
}

// ------------------- QR DATA FUNCTION -------------------
// Waits for QR code result from Serial (S, M, L) and sets rack counters
void QRdata(){
  stop();
  while(true){
    if(Serial.available()>0){
      String receivedData = Serial.readStringUntil('\n');
      if(receivedData=="S"){
        rkcounter=1;
        crkcounter =1;
        break;
      }
      else if(receivedData=="M"){
        rkcounter=2;
        crkcounter =2;
        break;
      }
      else if(receivedData=="L"){
        rkcounter=3;
        crkcounter =3;
        break;
      }
    }
  }
}

// ------------------- INTERSECTION MODULE -------------------
// Decides action at each intersection based on counter
void IntersectionModule(){
  if(counter == 1){
    forward();
    delay(300);
    pivotleft();
  }
  else if(counter == 2){
    forward();
    delay(400);
    pivotright();
  }
  else if(counter == 3){
    QRdata();
    stop();
    delay(1000);
    long dist = msrdist(); 
    while(dist >=6){
      dist = msrdist();
      forward();
    }
    stop();
    delay(1000);
    liftup();
    delay(500);
    backward();
    delay(2000);
    while(digitalRead(left) != 1 || digitalRead(right) != 1){
      backward();
    }
    stop();
    forward();
    delay(500);
    pivotleft();
  }
  else if(counter == 6){
    while(digitalRead(left) != 1 || digitalRead(right) != 1){
      linefollow();
    }
    stop();
    delay(1000);
    liftdown();
    delay(1000);
    backward();
    delay(400);
    reverseinter();
    stop();
    counter = 7;
  }
}

// ------------------- SORTING MODULES -------------------
// Navigate inside racks based on rkcounter value
void sortingRK(){
  if (rkcounter==1){
    forward();
    delay(300);
    pivotleft();
    stop();
    rkcounter=0;
    counter = 5;
  }
  else if (rkcounter==2){
    forward();
    delay(300);
    while(digitalRead(left) != 1 || digitalRead(right) != 1){
      linefollow();
    }
    stop();
    delay(300);
    rkcounter = 1;
    sortingRK();
  }
  else if (rkcounter==3){
    forward();
    delay(300);
    while(digitalRead(left) != 1 || digitalRead(right) != 1){
      linefollow();
    }
    stop();
    delay(300);
    rkcounter = 2;
    sortingRK();
  }
}

// Navigate inside cross racks based on crkcounter value
void sortingCRK(){
  if(crkcounter==1){
    stop();
    resetArduino();
  }
  else if(crkcounter==2){
    while(digitalRead(left) != 1 || digitalRead(right) != 1){
      linefollow();
    }
    stop();
    delay(300);
    crkcounter = 1;
    sortingCRK();
  }  
  else if(crkcounter==3){
    while(digitalRead(left) != 1 || digitalRead(right) != 1){
      linefollow();
    }
    stop();
    delay(300);
    crkcounter = 2;
    sortingCRK();
  }
}

// ------------------- REVERSE AT INTERSECTION -------------------
void reverseinter(){
  while(digitalRead(left) != 1 || digitalRead(right) != 1){
    backward();
  }
  stop();
  delay(1000);
  forward();
  delay(500);
  stop();
  delay(100);
  pivotleft();
}

// ------------------- LIFT CONTROL FUNCTIONS -------------------
// Lower lift until ultrasonic detects safe height
void liftdown(){
  long distn = dist();
  if(distn <= 9){
    while(distn < 10){
      distn = dist();
      digitalWrite(lin3,LOW);
      digitalWrite(lin4,HIGH);
      delay(200);
    }
    digitalWrite(lin3,LOW);
    digitalWrite(lin4,LOW);
  }
}

// Raise lift until ultrasonic detects top position
void liftup(){
  long distn = dist();
  if(distn >= 5){
    while(distn >= 4){
      distn = dist();
      digitalWrite(lin3,HIGH);
      digitalWrite(lin4,LOW);
    }
    digitalWrite(lin3,LOW);
    digitalWrite(lin4,LOW);
  }
}

// ------------------- ULTRASONIC DISTANCE FUNCTIONS -------------------
// Measure distance from lift ultrasonic sensor
long dist(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.0343 / 2;
  delay(100);
  return distance;
}

// Measure distance from front ultrasonic sensor
long msrdist(){
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);
  long duration = pulseIn(ECHO_PIN1, HIGH);
  long distance = duration * 0.0343 / 2;
  delay(100);
  return distance;
}

// ------------------- LINE FOLLOWING FUNCTION -------------------
void linefollow(){
  if(digitalRead(left) == 0 && digitalRead(right) == 0){
    slforward();
  }
  else if (digitalRead(left) == 0 && digitalRead(right) == 1){
    slright();
  }
  else if(digitalRead(left) == 1 && digitalRead(right) == 0){
    slleft();
  }
}

// ------------------- MOTOR CONTROL FUNCTIONS -------------------
void backward(){
  analogWrite(ena,adjust);
  analogWrite(enb,adjust);
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH);
}

void slforward(){
  analogWrite(ena,adjust);
  analogWrite(enb,adjust);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void forward(){
  analogWrite(ena, MOTOR_SPEED); 
  analogWrite(enb, MOTOR_SPEED);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void slleft(){
  analogWrite(ena, MOTOR_SPEED); 
  analogWrite(enb, MOTOR_SPEED);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void slright(){
  analogWrite(ena, MOTOR_SPEED); 
  analogWrite(enb, MOTOR_SPEED);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW);  
}

void stop(){
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// ------------------- RESET FUNCTION -------------------
// Forces Arduino to reset via watchdog timer
void resetArduino() {
  wdt_enable(WDTO_15MS);
  while (true) {
  }
}
