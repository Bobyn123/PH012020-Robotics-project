/*currenttly contains code for initiilisation of I/O pins, servo calibration, movement functions
  and object detection*/

#include <Servo.h>
#include <EEPROM.h>
//#define DEBUG CAL
#define DEBUG LINE
#define DEBUG IR
#define CAL

//const ints for LED pins
const int GREEN = 7;
const int YELLOW = 12;
const int RED = 13;
//const ints for push button pins
const int PBR = 2;
const int PBL = 4;
//ints for button state
int stateR = 0;
int stateL = 0;
//const ints for servo motors
const int RW = 5;
const int LW = 6;
//const ints for LDRs
const int LDRr = A0;
const int LDRm = A1;
const int LDRl = A2;
//LDR mid light values
int rightLDRav = EEPROM.read(16);
int midLDRav = EEPROM.read(13);
int leftLDRav = EEPROM.read(10);
//const ints for IR emitter and reciever pins
const int IRT = 3;
const int IRR = 2;
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

/* turns on infrared emitting diode pulsing at 38k, scans for reflection of pulsed IRED 10 times
  averages reading and returns 0 for object detected, returns 0 for not object detected */
int scan() {
  tone(IRT, 38000); // pulses IRED at 38k
  int scanState = 0; //initialise variable for averaging sensor readings
  delay(500);
  for (int i = 0; i < 10; i++) {
    if (digitalRead(IRR) == LOW) {
      scanState = 1;
    }
    else {
      scanState = 0;
    }
    Serial.println(scanState);
  }
  scanState = scanState % 10;
  Serial.println(scanState);
  noTone(IRT);
  return scanState;
}

//create left and right servo objects
Servo leftServo;
Servo rightServo;

/* calibration function for both servos sleft servo calibrated first then right
    to calibrate:
    observe left servo to see if it has stopped, if it has not stopped
    press left pushbutton to increment speed by 1, repeat untill left servo has stopped
    press right pushbutton to begin calibration of right servo and repeat proccess for right servo
    press right pushbutton to exit calibration */
void calServo () {
  int StopL = 90;
  int StopR = 90;
  leftServo.write(StopL);
  rightServo.write(StopR);
#ifdef DEBUG CAL
  Serial.println("Cal Left");
#endif
  while (true) {
    if (digitalRead(PBL) == LOW) {
      leftServo.write(StopL);
      waitKEY(PBL);
      StopL++;
#ifdef DEBUG CAL
      Serial.print("increment of L");
      Serial.println(StopL);
#endif
    }
    else if (digitalRead(PBR) == LOW) {
      waitKEY(PBR);
#ifdef DEBUG CAL
      Serial.println("End Cal Left");
      Serial.println("Cal Right");
#endif
      while (true) {
        if (digitalRead(PBL) == LOW) {
          rightServo.write(StopR);
          waitKEY(PBL);
          StopR++;
#ifdef DEBUG CAL
          Serial.print("increment of R");
          Serial.println(StopR);
#endif
        }
        else if (digitalRead(PBR) == LOW) {
          waitKEY(PBR);
#ifdef DEBUG CAL
          Serial.println("End Cal Right");
#endif
          break;
        }
      }
      break;
    }
  }
  EEPROM.update(0, StopL);
  EEPROM.update(1, StopR);
}

//function for calibrating light levels
void calLDR() {
  waitKEY(PBL);
  //  initilise ints to store average of light and dark values
  int lightR = 0;
  int darkR = 0;
  int lightM = 0;
  int darkM = 0;
  int lightL = 0;
  int darkL = 0;

  //  take 100 light level readings and sum readings
  for (int i = 0; i <= 25; i++) {
    //LDR readings
    int valR;
    int valM;
    int valL;
    valR = analogRead(LDRr);
    valM = analogRead(LDRm);
    valL = analogRead(LDRl);

#ifdef DEBUG CAL
    Serial.print("Val right ");
    Serial.println(valR);

    Serial.print("Val mid ");
    Serial.println(valM);

    Serial.print("Val left ");
    Serial.println(valL);
#endif

    lightR = lightR + valR;
    lightM = lightM + valM;
    lightL = lightL + valL;

  }

  waitKEY(PBR);

  // take 100 dark level readings and sum readings
  for (int i = 0; i <= 25; i++) {
    //LDR readings
    int valR;
    int valM;
    int valL;
    valR = analogRead(LDRr);
    valM = analogRead(LDRm);
    valL = analogRead(LDRl);

#ifdef DEBUG CAL
    Serial.print("Val right ");
    Serial.println(valR);

    Serial.print("Val mid ");
    Serial.println(valM);

    Serial.print("Val left ");
    Serial.println(valL);
#endif

    darkR = darkR + valR;
    darkM = darkM + valM;
    darkL = darkL + valL;

  }

  //  average light level readings
  lightR = lightR / 25;
  lightM = lightM / 25;
  lightL = lightL / 25;

  //  average dark level readings
  darkR = darkR / 25;
  darkM = darkM / 25;
  darkL = darkL / 25;

  //  taking average between light and dark

  rightLDRav = ((lightR + darkR) / 2);
  midLDRav = ((lightM + darkM) / 2);
  leftLDRav = ((lightL + darkL) / 2);

  EEPROM.update(16, rightLDRav);
  EEPROM.update(13, midLDRav);
  EEPROM.update(10, leftLDRav);

#ifdef DEBUG CAL
  Serial.print("Right average light ");
  Serial.println(rightLDRav);
  Serial.print("Mid average light ");
  Serial.println(midLDRav);
  Serial.print("Left average light ");
  Serial.println(leftLDRav);
#endif
  waitKEY(PBR);
  return;
}

//setspeed for both servos in format that has stop value at 0, forward is + backwars is -
void setspeed(int R, int L) {
  rightServo.write(EEPROM.read(1) - R);
  leftServo.write(EEPROM.read(0) + L);
}

//stops motors
void Halt() {
  setspeed(0, 0);
  delay(25);
}

//takes input distance in cm and travels that distance
void Forward(float cm) {
  int distanceTime = cm * distance;
  Halt();
  setspeed(90, 90);
  delay(distanceTime);
  Halt();
}

//same but for backwards
void Backward (float cm) {
  int distanceTime = cm * distance; //multiple distance entered by time required for one cm
  Halt();
  setspeed(-90, -90);
  delay(distanceTime);
  Halt();
}

//takes input in degrees and multiplies by time taken to turn one degree
void turnAngle(int deg) {
  float turnTime = (abs(deg) * turn);
#ifdef DEBUG
  Serial.println(deg);
  Serial.println(turnTime);
#endif
  // if angle is positive turn right
  if (deg < 0) {
    Halt();
    setspeed(45, -45);
    delay(turnTime);
    Halt();
  }
  //  if value is negative, turn left
  if (deg > 0) {
    Halt();
    setspeed(-45, 45);
    delay(turnTime);
    Halt();
  }
  else {
    return;
  }
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
  //set IR transmitter and receiver pins
  pinMode(IRT, OUTPUT);
  pinMode(IRR, INPUT);

#ifdef CAL
  calServo();
  Halt();
  setLED(0, 1, 1);
  calLDR();
#endif
  setLED(1, 0, 0);
}
void loop() {




  //    //  if object is found turn 180, else follow line
  //        if (scan()) {
  //            Halt();
  //  //          turnAngle(180);
  //    #ifdef DEBUG IR
  //            Serial.println("Obbstacle detected");
  //    #endif
  //          }
  //          else {
  //    #ifdef DEBUG IR
  //            Serial.println("No obstacle detected");
  //    #endif
  //middle LDR sees dark
  if (analogRead(LDRm) < midLDRav ) {
#ifdef DEBUG LINE
    Serial.println("forward");
#endif
    Forward(5);
  }

  // left LDR sees dark
  if ((analogRead(LDRl) < leftLDRav) || ((analogRead(LDRl) < leftLDRav) && (analogRead(LDRm) < midLDRav ))) {
#ifdef DEBUG LINE
    Serial.println("turn left");
#endif
    turnAngle(-10);
    Forward(5);
  }

  //right LDR sees dark
  if ((analogRead(LDRr) < rightLDRav) || ((analogRead(LDRr) < rightLDRav) && (analogRead(LDRm) < midLDRav ))) {
#ifdef DEBUG LINE
    Serial.println("turn right");
#endif
    turnAngle(10);
    Forward(5);
  }


}
