// ME210 Alyssa's Angels

#include <Arduino.h> 
#include <NewPing.h> //the superior Ultrasonic library
// #include <Servo.h>

/*-----------Module Defines-------------------------------------------------------*/
//Motors
#define enA_1 3
#define enA_2 11

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

//Connections to second Arduino (golfing mechanism)
#define positionPin 9
#define pressReleased 10


//non-changing variables
#define limit 1 //used in orienting L 
#define maxSensorRange 250 // cm

/*-----------Class Declarations-------------------------------------------------------*/
//Orientors
NewPing sonarL1(shareL1,shareL1, maxSensorRange);
NewPing sonarR1(shareR1,shareR1, maxSensorRange);
NewPing sonarL2(shareL2,shareL2, maxSensorRange);
NewPing sonarR2(shareR2,shareR2, maxSensorRange);

//Front 
NewPing stopper(shareStopper,shareStopper, maxSensorRange);

// //Servo
// Servo myservo;

/*-----------Function Prototypes-------------------------------------------------------*/
//Sensors
bool inRange(void);
void sensorTester(void);
bool crooked(void); //IMPORTANT
bool close (void); //IMPORTANT
void printStates(void);

//State Handlers
void handleOrientL(void); 
void handleMoveGP(void);
void handleMoveBP(void); 

void handleShoot(void);
void handleReturn(void); 

void handleHome(void);


//Fixers
void callFix(void);
void reorient(void); //IMPORTANT
void fixDistance(void); //IMPORTANT

//Motors and Movement
void goForward(void);
void goBackward(void);
void goLeft(void);
void goRight(void);
void turnLeft(void);
void turnRight(void);
void stopMotor(void);

/*-----------State Definitions-------------------------------------------------------*/
//i.e. Serial.print(STATE_IDLE) --> 0,  etc
typedef enum {STATE_IDLE, STATE_ORIENT_L, STATE_MOVEGP, STATE_MOVEBP,
STATE_SHOOT, STATE_RETURN, STATE_REORIENT, STATE_FD, STATE_HOME, STATE_STOP} States_t;

/*-----------Module Variables-------------------------------------------------------*/
int l1;
int r1;
int l2;
int r2;
int diff1;
int diff2; 
int counter; 
int maxR = 20; //in cm, max usable distance limit for orienting L
int cutoffDiff = 15; //in us

int stopDistance; 
int speed; 

int currTime;
int lastTime; 
// int startTime;

int inPosition = 0;
int pastInPosition = 0;
int lowerBound = 200; 
unsigned long golf_timer;

int stopCounter = 0; 
int exception = 0; 
int pastHomeTime = 0; 
int homeTime = 0; 

// int pos = 0;

States_t readState;
States_t storedState; 
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

  pinMode(positionPin, OUTPUT);
  pinMode(pressReleased, INPUT);

  digitalWrite(pressReleased, LOW);
  
  // pinMode(enA_2, OUTPUT);
  // pinMode(in5_1, OUTPUT);
  // pinMode(in6_2, OUTPUT);

  // myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  
  // analogWrite(enA_2, 0);
  // digitalWrite(in5_1, HIGH);
  // digitalWrite(in6_2, LOW);

  state = STATE_IDLE;

  // startTime = millis();
}

void loop() {
  currTime = millis(); 
  //printStates(); 

  switch (state) {
  case STATE_IDLE:
    state = STATE_ORIENT_L; 
    break;
  case STATE_ORIENT_L:
    speed = 65; 
    handleOrientL(); 
    break;

  case STATE_MOVEGP:
    speed = 125; 
    if (currTime - lastTime > 500) {
      l1 = sonarL1.ping(2*maxR);  // ping returns microseconds. 1 microsecond *58 = 1 cm, so 40 cm here. that's the max drift you want from wall. assumption. ping returns 0 if doesnt read anything/ timeout.
      r1 = sonarR2.ping(2*maxR);
      callFix(); 
    } else {
      handleMoveGP();
    }
    break;

  case STATE_MOVEBP:
    speed = 125;  
    if (currTime - lastTime > 500) {      // check every 0.5 seconds
      l1 = sonarL1.ping(2*maxR);
      r1 = sonarR2.ping(2*maxR);
      callFix();  
    } else {
      handleMoveBP(); 
    }
    break;

  case STATE_SHOOT:
    digitalWrite(positionPin, HIGH);
    handleShoot(); 
  //    if (digitalRead(pressReleased) == HIGH) {
  //   state = STATE_RETURN;
  //   digitalWrite(positionPin, LOW);     // no longer in position to shoot
  // }
    break;

  case STATE_RETURN:
    speed = 125;
    if (currTime - lastTime > 500) { //checks if need to fix
      l1 = sonarL1.ping(2*maxR);
      r1 = sonarR2.ping(2*maxR);
      callFix(); 
    } else {
      handleReturn(); 
    }
    break;

  case STATE_REORIENT:
    speed = 55;
    reorient(); 
    break; 
  case STATE_FD:
    speed = 75;
    fixDistance(); 
    break; 
  
  case STATE_HOME:
    homeTime = millis();  
    handleHome();
    break;

//TESTING state for now
  case STATE_STOP: 
    stopMotor(); 
    break;
  default: //uh oh moment
    Serial.println("wtf is going onnnnnn");
    stopMotor(); 
  }
}

/*-----------Functions Main Implementations-------------------------------------------------------*/

/*-----------Sensors-----------*/
bool inRange(void){
  if (l1 > 0 && r1 > 0 && l2 > 0 && r2 > 0) return true;
  return false; 
}

bool crooked(void){
  if ((abs(l1-r1) > lowerBound/2)) return true;
  return false; 
}

bool close(void){
  if (l1 < lowerBound || r1 < lowerBound) return true;
  return false; 
}

void printStates(void){
    if(state != readState){
    Serial.println(state);
    readState = state; 
  }
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

/*-----------States Handlers-------------------------------------------------------*/
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
      state = STATE_FD; 
      storedState = STATE_MOVEGP; 
    }
  } else {
    counter = 0;
  }
   turnRight();
}

void handleMoveGP(){
  stopDistance = stopper.ping(12);

  if(stopDistance > 0 && stopDistance < 250){
    stopMotor();  
    state = STATE_SHOOT;
  } else {
    goForward();
  }
}

void handleMoveBP(){
  l2 = sonarL2.ping(75);
  r2 = sonarR2.ping(75);

  if (l2 == 0 && r2 == 0){
    stopMotor();
    state = STATE_SHOOT;
  } else {
    goForward(); 
  }
}

void handleShoot(){
  if (digitalRead(pressReleased) == HIGH) {
    state = STATE_RETURN;
    digitalWrite(positionPin, LOW);     // no longer in position to shoot
  }
}

void handleReturn(){
  stopDistance = sonarL2.ping(12);

  if (stopDistance > 0 && stopDistance < 700){
    stopMotor();
    pastHomeTime = millis(); 
    state = STATE_HOME; 
  } else {
    goBackward(); 
  }
}

void handleHome(){
  if (homeTime > pastHomeTime + 4000){
    state = STATE_REORIENT;
    if (stopCounter % 2 == 0){
      storedState = STATE_MOVEBP;
    } else {
      storedState = STATE_MOVEGP;
    }
    stopCounter++;
  }
}

/*-----------Fixers-------------------------------------------------------*/
void callFix(){
  if (close()) {
    stopMotor(); 
    storedState = state;
    state = STATE_FD;
    exception = 1; 
  } else if (crooked()){
    stopMotor(); 
    storedState = state;
    state = STATE_REORIENT;
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
  if(l1 > 400 && r1 > 400){
    goRight(); 
  } else if (l1 < lowerBound || r1 < lowerBound){
    goLeft(); 
  } else { 
    if (exception == 1)
    {
      exception = 0; 
      state = STATE_REORIENT;
    } else {
      state = storedState;
    }
    lastTime = currTime; 
  }
}

/*-----------Motors and Movement-------------------------------------------------------*/
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

