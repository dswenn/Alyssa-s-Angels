// ME210 Alyssa's Angels

#include <Arduino.h> 
#include <HCSR04.h> //if missing look on PIO's directory, library is made by gamegine

/*-----------Module Defines-----------*/
#define enA_1 3
#define in1_1 2
#define in2_1 4

#define enB_2 9
#define in3_2 5
#define in4_2 6

#define enA_3 10
#define in1_3 7
#define in2_3 8

#define enB_4 11
#define in3_4 12
#define in4_4 13

#define speed 75 // 55 is min pwm at full charge w/o stalling...

#define TrigPin A1
#define limit 1 //used in orienting L 

/*-----------Class Declarations-----------*/
HCSR04 hcL(TrigPin, A2);
HCSR04 hcR(TrigPin, A3);
HCSR04 hcL2(TrigPin, A4);
HCSR04 hcR2(TrigPin, A5);

/*-----------Function Prototypes-----------*/
//Sensors
bool inRange(void);

//State Handlers
void handleOrientL(void); 

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
//add States as needed, separate with comma
typedef enum {STATE_IDLE, STATE_ORIENT_L, STATE_FORWARD, STATE_ORIENT_H, STATE_BOWL, 
STATE_REVERSE, STATE_HOME, STATE_STOP} States_t;

/*-----------Module Variables-----------*/
float l1;
float r1;
float l2;
float r2;
int counter; 
float maxR = 25.0; //in cm, max usable distance limit for orienting L
float diff = 0.5; //in cm, comparison value for difference between two sensors
States_t state; 

/*-----------MAIN CODE-----------*//*-----------MAIN CODE-----------*//*-----------MAIN CODE-----------*/
void setup() {
  Serial.begin(9600);
  Serial.println("Hello, world!");

  pinMode(enA_1, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);

  pinMode(enB_2, OUTPUT);
  pinMode(in3_2, OUTPUT);
  pinMode(in4_2, OUTPUT);

  pinMode(enA_3, OUTPUT);
  pinMode(in1_3, OUTPUT);
  pinMode(in2_3, OUTPUT);

  pinMode(enB_4, OUTPUT);
  pinMode(in3_4, OUTPUT);
  pinMode(in4_4, OUTPUT);

  state = STATE_IDLE;
}

void loop() {
  switch (state) {
  case STATE_IDLE:
    //start timer + other random stuff to do at start up
    state = STATE_ORIENT_L; 
    break;
  case STATE_ORIENT_L:
    handleOrientL(); 
    break;
  case STATE_FORWARD:
    //something 
    break;
  case STATE_ORIENT_H:
    //something 
    break;
  case STATE_BOWL:
    //something 
    break;
  case STATE_REVERSE:
    //something 
    break;
  case STATE_HOME:
    //something 
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
  if (l1 < maxR && r1 < maxR && l2 < maxR && r2 < maxR) return true;
  return false; 
}

/*-----------States Handlers-----------*/
void handleOrientL(){
  turnLeft();
  l1 = hcL.dist();
  r1 = hcR.dist();
  l2 = hcL2.dist();
  r2 = hcR2.dist();
  
  if(inRange()){
    if (abs(l1 - r1) < diff && abs(l2 - r2) < diff){
      counter++;
   } else {
      counter = 0;
    }
    if(counter == limit) {
      Serial.println(" achieved");
      counter = 0;
      state = STATE_STOP; 
      }
  } else {
    counter = 0;
  }
}

/*-----------Motors and Movement-----------*/
void goForward(){
  // Motor 1
  analogWrite(enA_1, speed); 
  digitalWrite(in1_1, HIGH);
  digitalWrite(in2_1, LOW);
  // Motor 2
  analogWrite(enB_2, speed); 
  digitalWrite(in3_2, HIGH);
  digitalWrite(in4_2, LOW);
  // Motor 3
  analogWrite(enA_3, speed); 
  digitalWrite(in1_3, HIGH);
  digitalWrite(in2_3, LOW);
  // Motor 4
  analogWrite(enB_4, speed); 
  digitalWrite(in3_4, HIGH);
  digitalWrite(in4_4, LOW);
  goForward(); 
}

void goBackward(){
  // Motor 1
  analogWrite(enA_1, speed); 
  digitalWrite(in1_1, LOW);
  digitalWrite(in2_1, HIGH);
  // Motor 2
  analogWrite(enB_2, speed); 
  digitalWrite(in3_2, LOW);
  digitalWrite(in4_2, HIGH);
  // Motor 3
  analogWrite(enA_3, speed); 
  digitalWrite(in1_3, LOW);
  digitalWrite(in2_3, HIGH);
  // Motor 4
  analogWrite(enB_4, speed); 
  digitalWrite(in3_4, LOW);
  digitalWrite(in4_4, HIGH);
}

void goRight(){
  // Motor 1
  analogWrite(enA_1, speed); 
  digitalWrite(in1_1, HIGH);
  digitalWrite(in2_1, LOW);
  // Motor 2
  analogWrite(enB_2, speed); 
  digitalWrite(in3_2, LOW);
  digitalWrite(in4_2, HIGH);
  // Motor 3
  analogWrite(enA_3, speed); 
  digitalWrite(in1_3, HIGH);
  digitalWrite(in2_3, LOW);
  // Motor 4
  analogWrite(enB_4, speed); 
  digitalWrite(in3_4, LOW);
  digitalWrite(in4_4, HIGH);
}

void goLeft(){
  // Motor 1
  analogWrite(enA_1, speed); 
  digitalWrite(in1_1, LOW);
  digitalWrite(in2_1, HIGH);
  // Motor 2
  analogWrite(enB_2, speed); 
  digitalWrite(in3_2, HIGH);
  digitalWrite(in4_2, LOW);
  // Motor 3
  analogWrite(enA_3, speed); 
  digitalWrite(in1_3, LOW);
  digitalWrite(in2_3, HIGH);
  // Motor 4
  analogWrite(enB_4, speed); 
  digitalWrite(in3_4, HIGH);
  digitalWrite(in4_4, LOW);
}

void turnRight(){
  // Motor 1
  analogWrite(enA_1, speed); 
  digitalWrite(in1_1, LOW);
  digitalWrite(in2_1, HIGH);
  // Motor 2
  analogWrite(enB_2, speed); 
  digitalWrite(in3_2, HIGH);
  digitalWrite(in4_2, LOW);
  // Motor 3
  analogWrite(enA_3, speed); 
  digitalWrite(in1_3, HIGH);
  digitalWrite(in2_3, LOW);
  // Motor 4
  analogWrite(enB_4, speed); 
  digitalWrite(in3_4, LOW);
  digitalWrite(in4_4, HIGH);
}

void turnLeft(){
  // Motor 1
  analogWrite(enA_1, speed); 
  digitalWrite(in1_1, HIGH);
  digitalWrite(in2_1, LOW);
  // Motor 2
  analogWrite(enB_2, speed); 
  digitalWrite(in3_2, LOW);
  digitalWrite(in4_2, HIGH);
  // Motor 3
  analogWrite(enA_3, speed); 
  digitalWrite(in1_3, LOW);
  digitalWrite(in2_3, HIGH);
  // Motor 4
  analogWrite(enB_4, speed); 
  digitalWrite(in3_4, HIGH);
  digitalWrite(in4_4, LOW);
}

void stopMotor(){
  // Motor 1
  analogWrite(enA_1, speed); 
  digitalWrite(in1_1, LOW);
  digitalWrite(in2_1, LOW);
  // Motor 2
  analogWrite(enB_2, speed); 
  digitalWrite(in3_2, LOW);
  digitalWrite(in4_2, LOW);
  // Motor 3
  analogWrite(enA_3, speed); 
  digitalWrite(in1_3, LOW);
  digitalWrite(in2_3, LOW);
  // Motor 4
  analogWrite(enB_4, speed); 
  digitalWrite(in3_4, LOW);
  digitalWrite(in4_4, LOW);
}

