/*currenttly contains code for initiilisation of I/O pins, servo calibration, movement functions
  and object detection*/



#include <Servo.h>
#include <EEPROM.h>
//#define DEBUG
//#define DEBUG_CAL
#define DEBUG_LDR
#define DEBUG_LINE
//#define DEBUG_IR
#define DEBUG_JUNCT

//#define CAL

//#define DANCE
//#define LINE
//#define OBSTACLE
#define JUNCT

// required to read EEPROM values for calibration settings
unsigned int readUIValue(int eepromAddress) {
  unsigned int uiVal;
  EEPROM.get(eepromAddress, uiVal);

  return uiVal;
}

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
const int rightThresh = readUIValue(16);
const  midThresh = readUIValue(13);
const leftThresh = readUIValue(10);
//const ints for IR emitter and reciever pins
const int IRT = 3;
const int IRR = 2;
//right and left servo stop values
const byte stopL = EEPROM.read(0);
const byte stopR = EEPROM.read(1);
//const ints related to wheel diamater and servo offsets
const int leftServoOffset = EEPROM.read(2);
const int rightServoOffset = EEPROM.read(3);
const int wheelDiameter = EEPROM.read(4);
//const ints for distance and turn time
const float timeForOneDegreeTurn = 16.3;
const int timeForOneCmRotation = 11780;

const int pi = 3.14159;

//set states for green, yellow, red LED
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

//waits for either or both pushbuttons, to set which pushbutton combination to wait for, set the appropriate int to 1 to wait for the button and 0 to not wait
byte checkKEY(byte leftButton, byte rightButton) {
  //  if waiting for left push button is desired
  byte leftButtonState = 0;
  byte rightButtonState = 0;

  while (true) {

    if (digitalRead(PBL) == HIGH && digitalRead(PBR) == HIGH) {
      leftButtonState = 0;
      rightButtonState = 0;
      delay(100);
      return 0;
    }
    else {

      if (digitalRead(PBL) == LOW) {
        leftButtonState = 1;
#ifdef DEBUG
        Serial.println(leftButtonState);
        Serial.println(leftButton);
#endif
        delay(20);
      }

      if (digitalRead(PBR) == LOW) {
#ifdef DEBUG
        Serial.println(rightButtonState);
        Serial.println(rightButton);
#endif
        rightButtonState = 1;
        delay(20);
      }

      if (leftButton == leftButtonState && rightButton == rightButtonState) {
          delay(100);
        return 1;
      }
    }
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

#ifdef CAL
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
  Serial.println("Cal light readings");
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
    unsigned int valR;
    unsigned int valM;
    unsigned int valL;
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
  Serial.println("Cal dark readings");
  waitKEY(PBL);

  // take 100 dark level readings and sum readings
  for (int i = 0; i <= 25; i++) {
    //LDR readings
    unsigned int valR;
    unsigned int valM;
    unsigned int valL;
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

  rightThresh = ((lightR + darkR) / 2);
  midThresh = ((lightM + darkM) / 2);
  leftThresh = ((lightL + darkL) / 2);

  EEPROM.update(16, rightThresh);
  EEPROM.update(13, midThresh);
  EEPROM.update(10, leftThresh);

#ifdef DEBUG CAL
  Serial.print("Right average light ");
  Serial.println(rightThresh);
  Serial.print("Mid average light ");
  Serial.println(midThresh);
  Serial.print("Left average light ");
  Serial.println(leftThresh);
#endif
  waitKEY(PBR);
  return;
}

#endif

#ifdef DEBUG_LDR

//function to check state of LDR reeadings against threshold and report if light or dark is seen reletive to refference
void debugLDR() {

  if  (analogRead(LDRl) < leftThresh) {
    Serial.println("left ldr sees dark");
  }
  if  (analogRead(LDRm) < midThresh ) {
    Serial.println("mid ldr sees dark");
  }
  if  (analogRead(LDRr) < rightThresh ) {
    Serial.println("right ldr sees dark");
  }
  if  (analogRead(LDRl) > leftThresh) {
    Serial.println("left ldr sees light");
  }
  if  (analogRead(LDRm) > midThresh ) {
    Serial.println("mid ldr sees light");
  }
  if  (analogRead(LDRr) > rightThresh ) {
    Serial.println("right ldr sees light");
  }
  delay(500);
}

#endif

//setspeed for both servos in format that has stop value at 0, forward is + backwars is -
void setspeed(int R, int L) {
  rightServo.write(stopR - R);
  leftServo.write(stopL + L);
}

//stops motors
void Halt() {
  setspeed(0, 0);
//  delay(25);
}

//takes input distance in cm and travels that distance
void Forward(float mm) {
  unsigned int distanceTime = (timeForOneCmRotation * ( mm / (wheelDiameter * pi)));
  Halt();
  setspeed(rightServoOffset, leftServoOffset);
  delay(distanceTime);
  Halt();
}

//same but for backwards
void Backward (float mm) {
  unsigned int distanceTime = (timeForOneCmRotation * ( mm / (wheelDiameter * pi))); //multiply distance entered by time required for one cm
  Halt();
  setspeed(-rightServoOffset, -leftServoOffset);
  delay(distanceTime);
  Halt();
}

//takes input in degrees and multiplies by time taken to turn one degree
void turnAngle(int deg) {
  float turnTime = (abs(deg) * timeForOneDegreeTurn);
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

//takes an input time in seconds and dances for that amount of time
void danceTime(unsigned int Time) {
  Time = Time * 1000;
  unsigned int start = millis();
  unsigned int endTime = start;
  while ((endTime - start) <= Time) {
    //    dance moves
    setLED(0, 0, 0);
    Forward(5);
    setLED(0, 1, 0);
    turnAngle(90);
    setLED(1, 0, 1);
    Backward(10);
    setLED(1, 0, 0);
    turnAngle(35);
    endTime = millis();
  }
}

//follows the black line using a zig zag method of reaquisition of the line
void lineFollow(int distanceGoal) {


  unsigned int distanceTravelled = 0;

  if (distanceGoal > 0) {
    //  goes forwards
    Serial.println("Follow Line Forwards");
    while (distanceTravelled < distanceGoal) {

      //  setLED(0, 0, 0);
      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* all three don't see line turn 1 degree untill line found */
        turnAngle(15);
        //    setLED(0, 0, 1);
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) < rightThresh )) { /* only right sees line hard turn right */
        turnAngle(10);
        //    setLED(0,1,0);
#ifdef DEBUG_LINE
        Serial.println("only right sees line hard turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* right and mid see line gentle turn right */
        turnAngle(5);
        //    setLED(0,1,0);
#ifdef DEBUG_LINE
        Serial.println("right and mid see line gentle turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* best case mid sees line left and right don't on track forwards */
        Forward(5);
        distanceTravelled += 3;
        //    setLED(0,1,0);
#ifdef DEBUG_LINE
        Serial.println("best case mid sees line left and right don't on track forwards");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* left and mid see line, light turn left */
        turnAngle(-5);
        //    setLED(0,1,0);
#ifdef DEBUG_LINE
        Serial.println("left and mid see line, light turn left");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* only left sees line hard turn left */
        turnAngle(-10);
        //    setLED(0,1,0);
#ifdef DEBUG_LINE
        Serial.println("only left sees line hard turn left ");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* all three see line scan for junction */
        Forward(5);
//        if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) {
//          return 1;
//        }
//        else {
//          return -1;
//        }
        //    setLED(0,1,0);
#ifdef DEBUG LINE
        Serial.println("all three sensors see black, start scanning for junction and move forward");
#endif
      }
    }
  }
  //goes backwards
  else {
    Serial.println("Follow Line Backwards");
    while (distanceTravelled < distanceGoal) {
      while (distanceTravelled < distanceGoal) {

        //  setLED(0, 0, 0);
        if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* all three don't see line turn 1 degree untill line found */
          turnAngle(15);
          //    setLED(0, 0, 1);
        }

        if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) < rightThresh )) { /* only right sees line hard turn right */
          turnAngle(10);
          //    setLED(0,1,0);
#ifdef DEBUG_LINE
          Serial.println("only right sees line hard turn right");
#endif
        }

        if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* right and mid see line gentle turn right */
          turnAngle(5);
          //    setLED(0,1,0);
#ifdef DEBUG_LINE
          Serial.println("right and mid see line gentle turn right");
#endif
        }

        if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* best case mid sees line left and right don't on track forwards */
          Backward(3);
          distanceTravelled += 3;
          //    setLED(0,1,0);
#ifdef DEBUG_LINE
          Serial.println("best case mid sees line left and right don't on track forwards");
#endif
        }

        if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* left and mid see line, light turn left */
          turnAngle(-5);
          //    setLED(0,1,0);
#ifdef DEBUG_LINE
          Serial.println("left and mid see line, light turn left");
#endif
        }

        if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* only left sees line hard turn left */
          turnAngle(-10);
          //    setLED(0,1,0);
#ifdef DEBUG_LINE
          Serial.println("only left sees line hard turn left ");
#endif
        }

        if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* all three see line scan for junction */
          Backward(1);
          //    setLED(0,1,0);
#ifdef DEBUG_LINE
          Serial.println("all three sensors see black, start scanning for junction and move forward");
#endif
        }

      }
    }
  }
}

//follows the line for a set number of junctions
void junctCount(int junctNumber) {
   int junctionCount = 0;
//  Serial.println(junctNumber);
//  Serial.println(junctionCount);
  Serial.println("start");
  while (junctionCount < junctNumber) {
    debugLDR();
    Serial.println("mid");
    if (junctDetect() == 1) {
      #ifdef DEBUG_JUNCT
    Serial.println("Junction found");
    setLED(1,0,0);
//    delay(500);
//    setLED(0,0,0);
    #endif
//    lineFollow(5);
//    delay(500);
    junctionCount++;
    }
    if (junctDetect() == -1) {
      #ifdef DEBUG_JUNCT
      Serial.println("no junction found");
      #endif
          setLED(0,0,1);
//    delay(500);
//    setLED(0,0,0);
//      lineFollow(1);
    }
    lineFollow(1);
    Serial.println(junctionCount);
  }
  Serial.println("end");
}

//checks to see if a junction is currently seen, returns 1 if junction is seen, -1 if not seen
int junctDetect() {

  int junctState = 0;

  for (int i = 0; i < 10; i++) {

    if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* best case mid sees line left and right don't on track forwards */

      junctState--;

#ifdef DEBUG JUNCT
      Serial.println("best case mid sees line left and right don't on track forwards");
#endif
    }

    if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* all three see line scan for junction */

      junctState++;

#ifdef DEBUG JUNCT
      Serial.println("all three sensors see black, start scanning for junction and move forward");
#endif
    }

  }
  if (junctState > 0) {
    return 1;
  }
  if (junctState < 0) {
    return -1;
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
//  calServo();
  Halt();
  setLED(0, 1, 1);
  calLDR();
#endif

#ifdef DEBUG
  Serial.println(PI);
  Serial.println(stopR);
  Serial.println(stopL);
  Serial.println(wheelDiameter);
  Serial.println(rightThresh);
  Serial.println(midThresh);
  Serial.println(leftThresh);
#endif  

  Halt();
  setLED(0, 0, 0);

}
void loop() {


#ifdef DANCE

KEY(1,0);
  //demonstration of straight line movement
//  Forward(50);
//  Backward(50);
//  //demonstraction of turn in place
//  turnAngle(90);
//  turnAngle(-90);
  //Dance time
  danceTime(20);

#endif



#ifdef LINE

if (checkKEY(0,1)) {
  Serial.println("follow the line");
  //current values of 75cm are definitly due to battery voltage replace with fresh by friday
  lineFollow(75);
  turnAngle(180);
  lineFollow(75);
  turnAngle(180);
}
else{}

#endif


#ifdef OBSTACLE

if (checkKEY(1,0)) {
  while (true){
  Serial.println("Follow the line and look for obstacles");
  if (scan()) {
    Serial.println("turn");
    turnAngle(180);
  }
  else {
    Serial.println("follow");
    lineFollow(1);
  }
}}
else{}

#endif

#ifdef JUNCT

waitKEY(PBL);
//if (checkKEY(1,1)) {
//  Serial.println("count the junctions and follow the line");
  junctCount(3);
//  Serial.println("counted specified number of junctions");
  turnAngle(180);
  junctCount(3);
//}
//else{}

#endif
}
