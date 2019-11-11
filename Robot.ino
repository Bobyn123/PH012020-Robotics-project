/*currenttly contains code for initiilisation of I/O pins, servo calibration, and movement functions
*/

#include <Servo.h>
#define DEBUG

//const ints for LED pins
const int GREEN = 7;
const int YELLOW = 12;
const int RED = 13;
//const ints for push button pins
const int PBR = 2;
const int PBL = 4;
//const ints for servo motors
const int RW = 5;
const int LW = 6;
//const ints for LDRs
const int LDRr = A0;
const int LDRm = A1;
const int LDRl = 2;
//const ints for IR emitter and reciever pins
const int IRT = 3;
const int IRR = 2;
//servo stop position int
int StopR = 90;
int StopL = 90;
//LDR readings
int val1 = analogRead(LDRr);
int val2 = analogRead(LDRm);
int val3 = analogRead(LDRl);
//const ints for distance and turn time
const float turn = 16.3;
const int distance = 93;

//set states for green, yellow, red LED
void setLED (int green_state, int yellow_state, int red_state) {
  digitalWrite(GREEN, green_state);
  digitalWrite(YELLOW, yellow_state);
  digitalWrite(RED, red_state);
}

//waits for left or right pushbutton
void waitKEY(int pin) {
  while (digitalRead(pin) == HIGH) {
    delay(20);
  }
  while (digitalRead(pin) == LOW) {
    delay(20);
  }
}

//attach left and right servo
Servo leftServo;
Servo rightServo;

//calibration function can probably simplify for arbitrary servo latter
void CalRight() {
  setLED(0, 0, 1);
  leftServo.write(StopL); //starts other servo at predefined value for voltage draw reasons
#ifdef DEBUG
  Serial.println("Cal Left");
#endif
  while (true) {
    //    if left pushbutton pressed increment servo stop value by 1 and start servo at new value
    if (digitalRead(PBL) == LOW) {
      rightServo.write(StopR);
      waitKEY(PBL);
      StopR++;
#ifdef DEBUG
      Serial.print("increment of R");
      Serial.println(StopR);
#endif
    }
    //    if right pushbutton pressed break from calibration loop
    else if (digitalRead(PBR) == LOW) {
      waitKEY(PBR);
      break;
    }
  }
#ifdef DEBUG
  Serial.println("End Cal Left");
#endif
}

//same as above but for left servo
void CalLeft() {
  setLED(0, 1, 1);
  rightServo.write(StopR);
#ifdef DEBUG
  Serial.println("Cal Right");
#endif
  while (true) {
    if (digitalRead(PBL) == LOW) {
      leftServo.write(StopL);
      waitKEY(PBL);
      StopL++;
#ifdef DEBUG
      Serial.print("increment of L");
      Serial.println(StopL);
#endif
    }
    else if (digitalRead(PBR) == LOW) {
      waitKEY(PBR);
      break;
    }
  }
#ifdef DEBUG
  Serial.println("End Cal Right");
#endif

}

//setspeed for both servos in format that has stop value at 0 forward is + backwars is -
void setspeed(int R, int L) {
  rightServo.write(StopR - R);
  leftServo.write(StopL + L);
}

//halt
void Halt() {
  setspeed(0, 0);
}

//takes input distance in cm and travels that distance
void Forward(float cm) {
  int distanceTime = cm * distance;
  Halt();
  delay(25);
  setspeed(90, 90);
  delay(distanceTime);
  Halt();
}

//same but for backwards
void Backward (float cm) {
  int distanceTime = cm * distance; //multiple distance entered by time required for one cm
  Halt();
  delay(25);
  setspeed(-90, -90);
  delay(distanceTime);
  Halt();
}

void turnAngle(int deg) {
  float turnTime = (abs(deg) * turn);
#ifdef DEBUG
  Serial.println(deg);
  Serial.println(turnTime);
#endif
  // if angle is positive turn right
  if (deg < 0) {
    Halt();
    delay(25);
    setspeed(45, -45);
    delay(turnTime);
    Halt();
  }
  //  if value is negative, turn left
  if (deg > 0) {
    Halt();
    delay(25);
    setspeed(-45, 45);
    delay(turnTime);
    Halt();
  }
  else {
    return;
  }
}

void Crab(float cm) {
  Forward(20*cm);
  turnAngle(45);
  Backward(5*cm);
  turnAngle(-15);
  Backward(10*cm);
  turnAngle(-30);
  Backward(8*cm);
}

void setup() {

  Serial.begin(9600);

  //set LED pins as output
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);
  //set push button pins as input
  pinMode(PBR, INPUT);
  pinMode(PBL, INPUT);
  //set servo pins as outputs
  pinMode(RW, OUTPUT);
  pinMode(LW, OUTPUT);
  //set LDR pins as inputs
  pinMode(LDRr, INPUT);
  pinMode(LDRm, INPUT);
  pinMode(LDRl, INPUT);
  //attach servos
  rightServo.attach(RW);
  leftServo.attach(LW);

  CalRight();
  CalLeft();
  setLED(1, 0, 0);
}
void loop() {
  // put your main code here, to run repeatedly:

  Halt();
  waitKEY(PBL);
  Crab(0.5);

}
