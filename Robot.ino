/*currenttly contains code for initiilisation of I/O pins, servo calibration, and movement functions
  and basic line following algorithm*/

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

void setspeed(int R, int L) {
  rightServo.write(StopR - R);
  leftServo.write(StopL + L);
}

void Halt() {
  setspeed(0, 0);
}

void Forward(int cm) {
  int d = cm * 93;
  setspeed(90, 90);
  Serial.println(d);
  delay(d);
}

void Backward (int cm) {
  int d = cm * 93;
  setspeed(-90, -90);
  Serial.println(d);
  delay(d);
}
//
//void turnAngle(deg){
//  
//}


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


waitKEY(PBR);
 setspeed(45, -45);
 waitKEY(PBL);Halt();

  //Serial.print(" Val right  ");
  //Serial.println(val1);
  //
  //Serial.print(" Val mid ");
  //Serial.println(val2);
  //
  //Serial.print(" Val left ");
  //Serial.println(val3);
  //
  //if ((val1 > val2) && (val3 > val2)) {
  //  Serial.println("Forward");
  //  Forward(90, 90);
  //  delay(500);
  //}
  //
  //if ((val2 > val1) && (val3 > val1)) {
  //  Serial.println("right");
  //  Rturn();
  //  delay(500);
  //}
  //
  //if((val2 > val3) && (val3 < val1)){
  //  Serial.println("left");
  //  Lturn();
  //  delay(500);
  //}


}
