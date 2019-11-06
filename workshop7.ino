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
//servo position values
int StopR = 90;
int StopL = 90;
const int RF = 180;
const int LF = 5;
const int RB = 5;
const int LB = 180;
// state for interrupt button
volatile byte state = HIGH;

void setLED (int green_state, int yellow_state, int red_state) {
  digitalWrite(GREEN, green_state);
  digitalWrite(YELLOW, yellow_state);
  digitalWrite(RED, red_state);
}

void waitKEY(int pin) {
  while (digitalRead(pin) == HIGH) {
    delay(20);
  }
  while (digitalRead(pin) == LOW) {
    delay(20);
  }
}

Servo leftServo;
Servo rightServo;

void CalRight() {
  setLED(0, 0, 1);
  leftServo.write(StopL);
#ifdef DEBUG
  Serial.println("Cal Left");
#endif
  while (true) {
    if (digitalRead(PBL) == LOW) {
      rightServo.write(StopR);
      waitKEY(PBL);
      StopR++;
#ifdef DEBUG
      Serial.print("increment of R");
      Serial.println(StopR);
#endif
    }
    else if (digitalRead(PBR) == LOW) {
     waitKEY(PBR);
      break;
    }
  }
#ifdef DEBUG
  Serial.println("End Cal Left");
#endif
}

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

void Halt() {
  rightServo.write(StopR);
  leftServo.write(StopL);
}

void Forward(int R, int L) {
  rightServo.write(R);
  leftServo.write(L);
  return;
}

void Backward (int R, int L) {
  rightServo.write(R);
  leftServo.write(L);
  return;
}

void Lturn() {
  rightServo.write(RF);
  leftServo.write(LB);
}

void Rturn() {
  rightServo.write(RB);
  leftServo.write(LF);
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

}
void loop() {
  // put your main code here, to run repeatedly:

  CalLeft();
  waitKEY(PBL);
  CalRight();
  setLED(1, 0, 0);

  Serial.println(StopR);
  Serial.println(StopL);

  while (true) {
    //
    //    Forward(0, 180);
    //    delay(1000);
    //    Halt();
    //    delay(500);
    //    Rturn();
    //    delay(1000);
    //    Backward(180, 0);
    //    delay(1000);
    Halt();
    delay(500);

  }
}
