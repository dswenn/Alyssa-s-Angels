// ME210 Alyssa's Angels

#include <Arduino.h>
#include <NewPing.h> //the best Ultrasonic library made by Tim Eckel

/* Some info about the NewPing library. Definitely recommend reading the full documentation.

.ping function returns microseconds (1 us * 58 = 1 cm OR 1 us * 150 = 1 in)
.ping_cm function returns rounded to nearest cm
.ping_in function returns rounded to nearest in
.ping type functions return 0 if times out (out of range) or too close to something
.ping type functions can accept a variable that gives it a timeout range, in cm

Library allows sharing of trigger and echo pins. Library also removes pulseIn() delay. 

*/


/*-----------Module Defines-------------------------------------------------------*/
//Motors Speed Control
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

//Connection to second Arduino
#define shoot 9

//non-changing variables
#define limit 1 //used in orienting L
#define maxSensorRange 250 // cm

/*-----------Class Declarations-------------------------------------------------------*/
//Orientors
NewPing sonarL1(shareL1,shareL1, maxSensorRange); //Side 1 is bottom wall
NewPing sonarR1(shareR1,shareR1, maxSensorRange);
NewPing sonarL2(shareL2,shareL2, maxSensorRange); //Side 2 is left wall
NewPing sonarR2(shareR2,shareR2, maxSensorRange);

//Front
NewPing stopper(shareStopper,shareStopper, maxSensorRange);

/*-----------Function Prototypes-------------------------------------------------------*/
//Testers
void printStates(void);
void sensorTester(void);

//Sensors
bool inRange(void);
bool crooked(void); 
bool close (void); 

//State Handlers
void handleOrientL(void);
void handleGP(void);
void handleBP(void);
void handleShoot(void);
void handleReturn(void);
void handleHome(void);

//Fixers
void callFix(void);
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

/*-----------State Definitions-------------------------------------------------------*/
typedef enum {STATE_IDLE, STATE_ORIENT_L, STATE_GP, STATE_BP,
STATE_SHOOT, STATE_RETURN, STATE_REORIENT, STATE_FD, STATE_HOME, STATE_STOP} States_t;

/*-----------Module Variables-------------------------------------------------------*/
//Sensor Variables
int l1;
int r1;
int l2;
int r2;
int diff1;
int diff2;
int counter;
int maxL = 15; //in cm, max distance limit for orienting L, based on STUDIO
int maxFD = 40; //in cm, max distance for fixing distance away from wall, based on testing
int cutoffDiff = 15; //microseconds
int upperBound = 600; //microseconds
int lowerBound = 450; //microseconds
int stopDistance;

//Motor Variables
int speed;

//State Variables
int checkInterval = 500; //milliseconds
int toggle = 0; //Allows movement between BP and GP

States_t readState;
States_t storedState;
States_t state;

//Time Variables
//All time related ints need to be large enough to hold game time. Thus, 32 bit. 
int32_t gametime = 130000;
int32_t currTime;

int32_t lastTime; //Used for checkInterval 

int32_t pastShootTime = 0;
int32_t shootTime = 0;

int32_t pastHomeTime = 0;
int32_t homeTime = 0;



/*-----------MAIN CODE-----------*//*-----------MAIN CODE-----------*//*-----------MAIN CODE-----------*/
void setup() {
  Serial.begin(9600);
  Serial.println("Hello, world! All aboard the Panda Express!");

  //Motors
  pinMode(enA_1, OUTPUT);

  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);

  pinMode(in3_2, OUTPUT);
  pinMode(in4_2, OUTPUT);

  pinMode(in1_3, OUTPUT);
  pinMode(in2_3, OUTPUT);

  pinMode(in3_4, OUTPUT);
  pinMode(in4_4, OUTPUT);

  //Communication to Arduino 2
  pinMode(shoot, OUTPUT);

  state = STATE_IDLE;

}

void loop() {
  printStates(); //Testing function - removed for actual game performance
  currTime = millis();

  if (currTime >= gametime){
      state = STATE_STOP;
  }

//STATE SWITCHING
  switch (state) {
  case STATE_IDLE:              //Beginning state, used for any testing or prelim actions
    state = STATE_ORIENT_L;
    break;
  case STATE_ORIENT_L:          //After turning on, the robot will orient itself
    speed = 55;
    handleOrientL();
    break;
  case STATE_GP:                //The robot will move forward to Good Press position
    speed = 125;
    if (currTime - lastTime > checkInterval) {
      callFix();
    } else {
      handleGP();
    }
    break;
  case STATE_BP:                //The robot will move forward to Bad Press position
    speed = 80;
    if (currTime - lastTime > checkInterval) { 
      callFix();
    } else {
      handleBP();
    }
    break;
  case STATE_SHOOT:            //Communicates to second Arduino to start shooting
    shootTime = currTime;
    digitalWrite(shoot, HIGH);
    handleShoot();
    break;
  case STATE_RETURN:           //Reverses to Studio
    speed = 125;
    if (currTime - lastTime > checkInterval) {  
      callFix();
    } else {
      handleReturn();
    }
    break;
  case STATE_REORIENT:        //State that is called periodically to fix drift      
    speed = 55;
    reorient();
    break;
  case STATE_FD:             //State that is called periodically to fix drift
    speed = 55;
    fixDistance();
    break;
  case STATE_HOME:           //Loading balls in Studio                      
    homeTime = currTime;
    handleHome();
    break;
  case STATE_STOP:          //End game and testing state
    digitalWrite(shoot, LOW);                    
    stopMotor();
    break;

  default:                  //uh oh moment
    Serial.println("damn. something is wrong.");
    stopMotor();
  }
}

/*-----------Functions Main Implementations-------------------------------------------------------*/

/*-----------Testing-----------*/
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

/*-----------Sensors-----------*/
bool inRange(void){
  if (l1 > 0 && r1 > 0 && l2 > 0 && r2 > 0) return true;
}

bool crooked(void){
  if ((abs(l1-r1) > cutoffDiff*2)) return true;
}

bool close(void){
  if (l1 < (lowerBound/2) || r1 < (lowerBound/2)) return true;
}

/*-----------States Handlers-------------------------------------------------------*/
void handleOrientL(){
  l1 = sonarL1.ping(maxL);      
  r1 = sonarR1.ping(maxL);
  l2 = sonarL2.ping(maxL);
  r2 = sonarR2.ping(maxL);

  diff1 = r1-l1;    // Bottom side ultrasonic distance sensors
  diff2 = l2-r2;    // Left side ultrasonic distance sensors

  /*  There are multiple ways we can dial in this:
  Changing "cutoffDiff" or "limit" or speed of turnLeft() */
  if(inRange()){
    if ((diff1 < cutoffDiff) && abs(diff2) < 300){        // cutoffDiff is the threshold range between two sensor readings 
      counter++;                    // The value controls how relaxed or tight the range for stopping is
      } else {
      counter = 0;
      }
        if(counter == limit) {
          stopMotor();
          counter = 0;
          state = STATE_FD;             // Fix distance first
          storedState = STATE_BP;
          return;
        }
  } else {
    counter = 0;
  }
   turnRight();
}

void handleGP(){
  stopDistance = stopper.ping(10);
  if (stopDistance < 580 && stopDistance > 0){
    stopMotor(); 
    storedState = STATE_SHOOT;
    state = STATE_REORIENT;
  } else {
    goForward(); 
  }

}
void handleBP(){
  l2 = sonarL2.ping(30);            // The value is the defined range. We want JUST outside studio.
  r2 = sonarR2.ping(30);

  if (l2 == 0 && r2 == 0){          // The moment the ultrasonic reads out of defined range, it returns 0
    stopMotor();
    storedState = STATE_SHOOT;
    state = STATE_REORIENT;            
  } else {
    goForward();                    // Keeps moving forward if doesn't reach out of range.
  }
}

void handleShoot(){
  if (shootTime > pastShootTime + 7000) { // 7 sec shooting? can change value
    digitalWrite(shoot, LOW);             // tells second Arduino to stop shoot 
    state = STATE_RETURN;
    lastTime = currTime; 
  }
}

void handleReturn(){
  stopDistance = sonarL2.ping(10);
  if (stopDistance < 580 && stopDistance > 0){ 
    stopMotor();
    pastHomeTime = currTime;
    state = STATE_HOME; 
  } else {
    goBackward();
  }
}

void handleHome(){

/*    For COMPETITION, all we did was make both storedStates STATE_BP, which is the less movement one.
Then we physically mounted our shooter at an angle to go into Good Press only. We changed the stopping
distance in STATE_BP to be just outside of the Studio.    */
  if (homeTime > pastHomeTime + 2500){ //Loading time
    toggle++; 
    state = STATE_REORIENT; 
    if (toggle % 2 == 1){  
      storedState = STATE_GP;
    } else {
      storedState = STATE_BP;
    } 
  }
}


/*-----------Fixers-------------------------------------------------------*/

/* Two cases of going wrong. Either it has drifted to be too close to the wall
OR it is a sufficient distance away, but not parallel to the wall AKA crooked. */
void callFix(){
l1 = sonarL1.ping(2*maxL);
r1 = sonarR2.ping(2*maxL);
  if (close()) {
    stopMotor();
    state = STATE_FD;
  } else if (crooked()){
    stopMotor();
    state = STATE_REORIENT;
  } else {
    lastTime = currTime; 
  }

//Ensures after "fixing" drift, robot returns to what it was doing or goes to the state you wanted it to
storedState = state; 
}

void reorient(){                                          
  l1 = sonarL1.ping(maxFD);
  r1 = sonarR1.ping(maxFD);
   if(abs(r1-l1) > 3*cutoffDiff && l1 > 0 && r1 > 0){     // Bottom sensors are reading 
    if(l1 < r1){
      turnRight();                                        // Turn based on sensor readings
    } else {
      turnLeft();
    }
  } else {                                                // Bottom sensors values are close to each other
    state = STATE_FD;
  }
}

void fixDistance(){                                       //Goes up or down to desired position based on some tolerance band
  l1 = sonarL1.ping(maxFD);
  r1 = sonarR1.ping(maxFD);
  if(l1 > upperBound && r1 > upperBound){                 //Bottom sensors
    goRight();                                            //Towards wall
  } else if (l1 < lowerBound && r1 < lowerBound){         // Goes up
    goLeft();                                             //Away from wall
  } else {
    stopMotor(); 
    state = storedState; //Returns to movement state that it previously was at
    lastTime = currTime;
    pastShootTime = currTime; 
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
