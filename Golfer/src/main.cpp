#include <Arduino.h>
#include <Servo.h>

#define PWM_PinA 11
#define in1 4
#define in2 5

#define positionPin 2
#define pressReleased 3

Servo myservo; 

// Robot position reads
int inPosition;
int pastInPosition = 0;

//Timers
int currTime;
int startTime;
unsigned long golf_timer;
unsigned long game_time = 130000;
//const int interval = 20;

// Servo positions
// int oldPos = 0;
// int newPos = 1;
// int pos = 0;

// Communication with main Arduino
int signalDone = 0;
int pressReleaseTime = 8500;          // can change this based on the speed of the motor. Ideally it's tuned so that we get exactly 4 spins

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Hello, world!");

  pinMode(PWM_PinA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  analogWrite(PWM_PinA, 0);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  startTime = millis();
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(180);     // tell servo to go to 180 degrees
  
}

void loop() {
  currTime = millis(); 

  if(currTime - startTime >= game_time && !signalDone) {      // Checking if the game timer has expired
    myservo.write(0);     // tell servo to go to position
    signalDone = 1;

    // IF THE ABOVE SERVO CODE DOESN'T WORK, TRY THIS (it just uses delays but that's ok for the end of the game)



    // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    //   // in steps of 1 degree
    //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //   delay(15);                       // waits 15ms for the servo to reach the position
    // }
    // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //   delay(15);                       // waits 15ms for the servo to reach the position
    // }
  } 

  inPosition = digitalRead(positionPin);        // check other Arduino to see if it's time to shoot/golf

  if (inPosition && inPosition != pastInPosition && !signalDone) {       // send commands only when the reading on the pin changes
    analogWrite(PWM_PinA, 255);           // write high
    golf_timer = millis();                // start a timer for the golfing
  }

  if (((inPosition && currTime >= (golf_timer + pressReleaseTime)) || !inPosition) && !signalDone) {
    analogWrite(PWM_PinA, 0);             // stop golfing
    digitalWrite(pressReleased, HIGH);    // tell other Arduino we are done 
  }

  pastInPosition = inPosition;

}