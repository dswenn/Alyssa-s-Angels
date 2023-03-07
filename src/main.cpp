// ME210 Alyssa's Angels

#include <Arduino.h> 
#include <NewPing.h> //the superior Ultrasonic library

/*-----------Module Defines-----------*/
#define enA_1 3

//Motor 1
#define in1_1 2
#define in2_1 4

//Motor 2
#define in3_2 5
#define in4_2 6

//Motor 3
#define in1_3 7
#define in2_3 8

//Motor 4
#define in3_4 12
#define in4_4 13

//Ultrasonic Sensors
#define shareStopper A1
#define shareL1 A2
#define shareR1 A3
#define shareL2 A4
#define shareR2 A5


//non-changing variables
#define limit 1 //used in orienting L 
#define maxSensorRange 250

/*-----------Class Declarations-----------*/
//Orientors
NewPing sonarL1(shareL1,shareL1, maxSensorRange);
NewPing sonarR1(shareR1,shareR1, maxSensorRange);
NewPing sonarL2(shareL2,shareL2, maxSensorRange);
NewPing sonarR2(shareR2,shareR2, maxSensorRange);

//Front 
NewPing stopper(shareStopper,shareStopper, maxSensorRange);

/*-----------Function Prototypes-----------*/
//Sensors
bool inRange(void);
void sensorTester(void);
bool crooked(void);

//State Handlers
void handleOrientL(void); 
void handleForward(void);
void handleReverse(void);
void handleOrientH(void);
void handleFixH(void); 
void handleShoot(void);
void handleHome(void); 

//Fixers
void reorient(void);
void fixDistance(void);

//Motors and Movement
void goForward(void);
void goBackward(void);
void goLeft(void);
void goRight(void);
void turnLeft(void);
void turnRight(void);
void stopMotor(void);

/*-----------State Definitions-----------*/
//if you print out the states, they correspond to their position in the array
//i.e. Serial.print(STATE_IDLE) --> 0,  etc
typedef enum {STATE_IDLE, STATE_ORIENT_L, STATE_FORWARD, STATE_ORIENT_H, STATE_FIX_H, STATE_SHOOT, 
STATE_REVERSE, STATE_HOME, STATE_REORIENT, STATE_FD, STATE_STOP} States_t;

/*-----------Module Variables-----------*/
int l1;
int r1;
int l2;
int r2;
int diff1;
int diff2; 
int counter; 
int direction = 1; //for fix_h
int maxR = 20; //in cm, max usable distance limit for orienting L
int cutoffDiff = 10; //in us

int stopDistance; 
int speed; 

int currTime;
int lastTime; 

States_t readState;
States_t lastState; 
States_t state; 

/*-----------MAIN CODE-----------*//*-----------MAIN CODE-----------*//*-----------MAIN CODE-----------*/
void setup() {
  Serial.begin(9600);
  Serial.println("Hello, world! All aboard the Panda Express!");

  pinMode(enA_1, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);

  pinMode(in3_2, OUTPUT);
  pinMode(in4_2, OUTPUT);

  pinMode(in1_3, OUTPUT);
  pinMode(in2_3, OUTPUT);

  pinMode(in3_4, OUTPUT);
  pinMode(in4_4, OUTPUT);

  state = STATE_IDLE;
}

void loop() {
  if(state != readState){
    Serial.println(state);
    readState = state; 
  }
  currTime = millis(); 
  switch (state) {
  case STATE_IDLE:
    //start timer + other random stuff to do at start up
    //Serial.println(stopper.ping());
    //delay(1000);
    state = STATE_ORIENT_L; 
    //sensorTester(); 
    break;
  case STATE_ORIENT_L:
    speed = 60; 
    handleOrientL(); 
    break;
  case STATE_FORWARD:
    speed = 150; 
    handleForward();
    if (currTime - lastTime > 1500 && crooked()) {
      stopMotor(); 
      lastState = state;
      state = STATE_REORIENT; 
    }
    break;
  case STATE_ORIENT_H:
    speed = 55; 
    handleOrientH(); 
    break;
  case STATE_FIX_H:
    speed = 75; 
    handleFixH();
    break; 
  case STATE_SHOOT:
    handleShoot(); 
    //state = STATE_STOP;
    break;
  case STATE_REVERSE:
    speed = 150; 
    handleReverse();  
    if (currTime - lastTime > 1000 && crooked()) {
      stopMotor(); 
      lastState = state;
      state = STATE_REORIENT; 
    }
    break;
  case STATE_HOME:
    speed = 150;
    handleHome(); 
    break;
  case STATE_REORIENT:
    //speed = 75;
    reorient(); 
    break; 
  case STATE_FD:
    speed = 75;
    fixDistance(); 
    break; 
  case STATE_STOP: //TESTING state for now
    stopMotor(); 
    break;
  default: //uh oh moment
    Serial.println("wtf is going onnnnnn");
    stopMotor(); 
  }
}

/*-----------Functions Main Implementations-----------*/

/*-----------Sensors-----------*/
bool inRange(void){
  if (l1 > 0 && r1 > 0 && l2 > 0 && r2 > 0) return true;
  return false; 
}

bool crooked(void){
  l1 = sonarL1.ping(2*maxR);
  r1 = sonarR2.ping(2*maxR);

  if (abs(l1-r1) > 120) return true;
  return false; 
}

/*-----------States Handlers-----------*/
void handleOrientL(){
  l1 = sonarL1.ping(maxR);
  r1 = sonarR1.ping(maxR);
  l2 = sonarL2.ping(maxR);
  r2 = sonarR2.ping(maxR);
  diff1 = r1-l1; 
  diff2 = l2-r2; 
  
  //there are multiple ways we can dial in this:
  //changing "cutoffDiff" or "limit" or speed of turnLeft()
  if(inRange()){
    if (diff1 < cutoffDiff){
      counter++;
   } else {
      counter = 0;
    }
    if(counter == limit) {
      stopMotor();
      counter = 0; 
      state = STATE_FIX_H; 
      lastTime = currTime; 
      //state = STATE_STOP; // use for testing
    }

  } else {
    counter = 0;
  }
   turnRight();
}

void handleForward(){
  stopDistance = stopper.ping(12);

  if(stopDistance > 0 && stopDistance < 700){
    stopMotor(); 
    delay(1000);
    lastTime = currTime + 1000; 
    state = STATE_REVERSE;
  } else {
    goForward();
  }
}

void handleReverse(){
  //stopDistance = sonarR2.ping_cm(95);
  l2 = sonarL2.ping(95);
  r2 = sonarR2.ping(95);
  int val = 3200;

  if (l2 > 0 && l2 < val && r2 > 0 && r2 < val){
    stopMotor();
    delay(1000);
    state = STATE_HOME;
  } else {
    goBackward(); 
  }
}

void handleOrientH(){
  l1 = sonarL1.ping(3*maxR);
  r1 = sonarR1.ping(3*maxR);

  if(abs(l1-r1) > cutoffDiff && l1 > 0 && r1 > 0){
    if(l1 < r1){
      turnRight();
    } else {
      turnLeft();
    }
  } else {
    stopMotor(); 
    state = STATE_FIX_H;
  }
}

void handleFixH(){
  l1 = sonarL1.ping(2*maxR);
  r1 = sonarR1.ping(2*maxR);
  if (l1 > 300 && r1 > 300) {
    goRight(); 
  } else {
    stopMotor(); 
    if (direction == 1)
    {
      state = STATE_FORWARD;
      direction = 0; 
    } else {
      state = STATE_SHOOT; 
      direction = 1;
    }
    
  }
}

void handleShoot(){
  state = STATE_REVERSE; 
}

void handleHome(){
  l2 = sonarL2.ping(20);
  r2 = sonarR2.ping(20);
  int val = 300;

  if (l2 > 0 && l2 < val && r2 > 0 && r2 < val){
    stopMotor();
    delay(1000);
    state = STATE_ORIENT_L;
  } else {
    goBackward(); 
  }
}

void reorient(){
  l1 = sonarL1.ping(maxR);
  r1 = sonarR1.ping(maxR);
   if(abs(r1-l1) > 2*cutoffDiff && l1 > 0 && r1 > 0){
    if(l1 < r1){
      turnRight();
    } else {
      turnLeft();
    }
  } else {
    state = STATE_FD; 
  }
}

void fixDistance(){
  l1 = sonarL1.ping(3*maxR);
  r1 = sonarR1.ping(3*maxR);
  if(l1 > 350 && r1 > 350){
    goRight(); 
  } else if (l1 < 100 && r1 < 100){
    goLeft(); 
  } else {
    state = lastState; 
    lastTime = currTime; 
  }
}

/*-----------Motors and Movement-----------*/
void goForward(){
  analogWrite(enA_1, speed); 
  //Motor 1
  digitalWrite(in1_1, HIGH);
  digitalWrite(in2_1, LOW);
  // Motor 2
  digitalWrite(in3_2, HIGH);
  digitalWrite(in4_2, LOW);
  // Motor 3
  digitalWrite(in1_3, HIGH);
  digitalWrite(in2_3, LOW);
  // Motor 4
  digitalWrite(in3_4, HIGH);
  digitalWrite(in4_4, LOW);
}

void goBackward(){
  analogWrite(enA_1, speed); 
  //Motor 1
  digitalWrite(in1_1, LOW);
  digitalWrite(in2_1, HIGH);
  // Motor 2
  digitalWrite(in3_2, LOW);
  digitalWrite(in4_2, HIGH);
  // Motor 3
  digitalWrite(in1_3, LOW);
  digitalWrite(in2_3, HIGH);
  // Motor 4
  digitalWrite(in3_4, LOW);
  digitalWrite(in4_4, HIGH);
}

void goLeft(){
  analogWrite(enA_1, speed);
  // Motor 1 
  digitalWrite(in1_1, HIGH);
  digitalWrite(in2_1, LOW);
  // Motor 2
  digitalWrite(in3_2, LOW);
  digitalWrite(in4_2, HIGH);
  // Motor 3
  digitalWrite(in1_3, HIGH);
  digitalWrite(in2_3, LOW);
  // Motor 4
  digitalWrite(in3_4, LOW);
  digitalWrite(in4_4, HIGH);
}

void goRight(){
  analogWrite(enA_1, speed); 
  // Motor 1
  digitalWrite(in1_1, LOW);
  digitalWrite(in2_1, HIGH);
  // Motor 2
  digitalWrite(in3_2, HIGH);
  digitalWrite(in4_2, LOW);
  // Motor 3
  digitalWrite(in1_3, LOW);
  digitalWrite(in2_3, HIGH);
  // Motor 4
  digitalWrite(in3_4, HIGH);
  digitalWrite(in4_4, LOW);
}

void turnRight(){
  analogWrite(enA_1, speed);
  // Motor 1 
  digitalWrite(in1_1, LOW);
  digitalWrite(in2_1, HIGH);
  // Motor 2
  digitalWrite(in3_2, HIGH);
  digitalWrite(in4_2, LOW);
  // Motor 3 
  digitalWrite(in1_3, HIGH);
  digitalWrite(in2_3, LOW);
  // Motor 4
  digitalWrite(in3_4, LOW);
  digitalWrite(in4_4, HIGH);
}

void turnLeft(){
  analogWrite(enA_1, speed); 
  //Motor 1
  digitalWrite(in1_1, HIGH);
  digitalWrite(in2_1, LOW);
  // Motor 2
  digitalWrite(in3_2, LOW);
  digitalWrite(in4_2, HIGH);
  // Motor 3
  digitalWrite(in1_3, LOW);
  digitalWrite(in2_3, HIGH);
  // Motor 4 
  digitalWrite(in3_4, HIGH);
  digitalWrite(in4_4, LOW);
}

void stopMotor(){
  analogWrite(enA_1, LOW); 
  //Motor 1
  digitalWrite(in1_1, LOW);
  digitalWrite(in2_1, LOW);
  // Motor 2
  digitalWrite(in3_2, LOW);
  digitalWrite(in4_2, LOW);
  // Motor 3
  digitalWrite(in1_3, LOW);
  digitalWrite(in2_3, LOW);
  // Motor 4
  digitalWrite(in3_4, LOW);
  digitalWrite(in4_4, LOW);
}

void sensorTester(){ 
  Serial.print("l1: ");
  Serial.println(sonarL1.ping());
   Serial.print("r1: ");
  Serial.println(sonarR1.ping());
   Serial.print("l2: ");
  Serial.println(sonarL2.ping());
   Serial.print("r2: ");
  Serial.println(sonarR2.ping());
   delay(1000);


}

