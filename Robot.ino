/*finished assingment code, contains functions for calibration of servos motor stop values, LDR light averages
  basic line following, object detection and junction counting,
  options to perform assingment stages 2, 3 and 4 using button presses*/

/* BASIC USER GUIDE
  define CAL to allow for calibration of servos and LDR light averages
  to calibrate servos:
            left servo calibrated first then right
    observe left servo to see if it has stopped, if it has not stopped
    press left pushbutton to increment speed by 1, repeat untill left servo has stopped
    press right pushbutton to begin calibration of right servo and repeat proccess for right servo
    press right pushbutton to exit calibration
  to calibrate LDR:
      light level is recorded first, then dark level
    place sensors over white paper and press left push button, wait untill all LEDS flash for 1 second
    place sensors over black paper and press left push button, wait untill all LEDS flash for 1 second and LDR calibration will be complete
*/

#include <Servo.h>
#include <EEPROM.h>

#define DEBUG
#define DEBUG_CAL
#define DEBUG_LDR
#define DEBUG_LINE
#define DEBUG_IR
#define DEBUG_JUNCT

#define CAL

#define DANCE
#define LINE
#define OBSTACLE
#define JUNCT

// required to read EEPROM values for calibration settings
unsigned int readUIValue(int eepromAddress) {
  unsigned int uiVal;
  EEPROM.get(eepromAddress, uiVal);

  return uiVal;
}

//constant byte for storing pi value to be used to calculate turntime to travel x distance
const byte pi = 3.14159;
//const byte for LED pins
const byte GREEN = 7;
const byte YELLOW = 12;
const byte RED = 13;
//const byte for push button pins
const byte PBR = 2;
const byte PBL = 4;
//bytes for button state
byte stateR = 0;
byte stateL = 0;
//const ints for servo motor pins
const byte RW = 5;
const byte LW = 6;
//const byte for LDRs
const byte LDRr = A0;
const byte LDRm = A1;
const byte LDRl = A2;
//LDR average refference value read using readUIvalue function
const byte rightThresh = readUIValue(16);
const byte midThresh = readUIValue(13);
const byte leftThresh = readUIValue(10);
//const ints for IR emitter and reciever pins
const int IRT = 3;
const int IRR = 2;
//const right and left servo stop values read from EEPROM values
const byte stopL = EEPROM.read(0);
const byte stopR = EEPROM.read(1);
//const byte related to wheel diamater and servo offsets
const byte leftServoOffset = EEPROM.read(2);
const byte rightServoOffset = EEPROM.read(3);
const byte wheelDiameter = EEPROM.read(4);
//const float and int for time taken to move 1cm or turn 1 degree
const float timeForOneDegreeTurn = 16.3;
const int timeForOneCmRotation = 11780;
//const int for wheel circumference
const int wheelCircumference = (wheelDiameter * pi);

//set states for green, yellow, red LED input 1 to turn LED on 0 to turn LED off
void setLED (int greenState, int yellowState, int redState) {
  digitalWrite(GREEN, greenState);
  digitalWrite(YELLOW, yellowState);
  digitalWrite(RED, redState);
}

//function that loops untill desired key is pressed and depressed, to set the key to wait for enter PBL(Push button left) or PBR(Push button right)
void waitKEY(int pin) {
  while (digitalRead(pin) == HIGH) {
    delay(20);
  }
  while (digitalRead(pin) == LOW) {
    delay(20);
  }
}

//checks if either or both pushbuttons are pressed, to set which pushbutton combination to wait for, set the appropriate int to 1 to wait for the button and 0 to not wait
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

//takes input of desired state for green, yellow, red, LED and duration of time to flash LEDs for
void flashLED(int greenState, int yellowState, int redState, unsigned int Time) {
  Time = Time * 1000;
  unsigned int start = millis();
  unsigned int endTime = start;
  //  while current duration of loop is smaller than desired duration of loop continue loop
  while ((endTime - start) <= Time) {
    setLED(greenState, yellowState, redState);
    delay(50);
    setLED(0, 0, 0);
    endTime = millis();
  }
}

/* turns on infrared emitting diode pulsing at 38k, scans for reflection of pulsed IRED 10 times
  averages reading and returns 1 for object detected, returns 0 for no object detected and then
  turns off infrared emitting diode */
int scan() {
  tone(IRT, 38000); // pulses IRED at 38k
  int scanState = 0; //initialise variable for averaging sensor readings
  delay(500);
  for (byte i = 0; i < 10; i++) {
    if (digitalRead(IRR) == LOW) {
      scanState = 1;
    }
    else {
      scanState = 0;
    }
#ifdef DEBUG_IR
    Serial.println(scanState);
#endif
  }
  scanState = scanState % 10;
#ifdef DEBUG_IR
  Serial.println(scanState);
#endif
  noTone(IRT);
  return scanState;
}


//create left and right servo objects
Servo leftServo;
Servo rightServo;

//if CAL is not defined, calibration functions will not be compiled onto the arduino
#ifdef CAL
/* calibration function for both servos left servo calibrated first then right
    to calibrate:
    observe left servo to see if it has stopped, if it has not stopped
    press left pushbutton to increment speed by 1, repeat untill left servo has stopped
    press right pushbutton to begin calibration of right servo and repeat proccess for right servo
    press right pushbutton to exit calibration */
void calServo () {
  //  bytes to store new stop values, 90 is used as it was always below the actual stop value
  byte StopLeft = 90;
  byte StopRight = 90;

  //writes both right and left servos to begin using current stop values
  leftServo.write(StopLeft);
  rightServo.write(StopRight);

  Serial.println("Cal Left");

  while (true) {
    //    if left push button is pressed, increment left stop value by one and write new stop value to left servo
    if (digitalRead(PBL) == LOW) {
      waitKEY(PBL); //waits for left push button to be fully pressed to prevent debounce
      StopLeft++;
      leftServo.write(StopLeft);
#ifdef DEBUG_CAL
      Serial.print("increment of L");
      Serial.println(StopLeft);
#endif
    }
    //    if right push button pressed, calibration of left servo is ended, contains nested loop of right servo calibration for flow control
    else if (digitalRead(PBR) == LOW) {
      waitKEY(PBR); //waits for right push button to be fully pressed to prevent debounce

      Serial.println("End Cal Left");
      Serial.println("Cal Right");

      //    if left push button is pressed, increment right stop value by one and write new stop value to right servo
      while (true) {
        if (digitalRead(PBL) == LOW) {
          rightServo.write(StopRight);
          waitKEY(PBL);
          StopRight++;
#ifdef DEBUG_CAL
          Serial.print("increment of R");
          Serial.println(StopRight);
#endif
        }
        else if (digitalRead(PBR) == LOW) {
          waitKEY(PBR);

          Serial.println("End Cal Right");
          break;
        }
      }
      break;
    }
  }

  //  updating EEPROM left and right servo stop values
  EEPROM.update(0, StopLeft);
  EEPROM.update(1, StopRight);
}

/*function for calibrating left, middle, and right LDR average light levels
  place sensors over white paper and press left push button, wait untill all LEDS flash for 1 second
  place sensors over black paper and press left push button, wait untill all LEDS flash for 1 second and LDR calibration will be complete
*/
void calLDR() {
  Serial.println("Cal light readings");
  waitKEY(PBL); //press left button to begin calibration of light level sensor readings
  //  initilise ints to store average of light and dark values
  unsigned int lightR = 0;
  unsigned int darkR = 0;
  unsigned int lightM = 0;
  unsigned int darkM = 0;
  unsigned int lightL = 0;
  unsigned int darkL = 0;
  unsigned int rightThreshold = 0;
  unsigned int midThreshold = 0;
  unsigned int leftThreshold = 0;

  //  take 25 light level readings and sum readings from each loop
  for (byte i = 0; i <= 25; i++) {

    lightR += analogRead(LDRr);
    lightM += analogRead(LDRm);
    lightL += analogRead(LDRl);

#ifdef DEBUG CAL
    Serial.print("Val right ");
    Serial.println(analogRead(LDRr));

    Serial.print("Val mid ");
    Serial.println(analogRead(LDRm));

    Serial.print("Val left ");
    Serial.println(analogRead(LDRl));
#endif

  }

  flashLED(1, 1, 1, 1);
  Serial.println("Cal dark readings");
  waitKEY(PBL); // push right button to begin calibration of dark level sensor readings

  // take 25 dark level readings and sum readings from each loop
  for (byte i = 0; i <= 25; i++) {

    darkR += analogRead(LDRr);
    darkM += analogRead(LDRm);
    darkL += analogRead(LDRl);

#ifdef DEBUG CAL
    Serial.print("Val right ");
    Serial.println(analogRead(LDRr));

    Serial.print("Val mid ");
    Serial.println(analogRead(LDRm));

    Serial.print("Val left ");
    Serial.println(analogRead(LDRl));
#endif

  }

  //  average light level readings
  lightR /= 25;
  lightM /= 25;
  lightL /= 25;

  //  average dark level readings
  darkR /= 25;
  darkM /= 25;
  darkL /= 25;

  //  taking average between value light and dark readings

  rightThreshold = ((lightR + darkR) / 2);
  midThreshold = ((lightM + darkM) / 2);
  leftThreshold = ((lightL + darkL) / 2);

  //  update EEPROM values to store new LDR threshold calibrated values
  EEPROM.update(16, rightThreshold);
  EEPROM.update(13, midThreshold);
  EEPROM.update(10, leftThreshold);
  //prints current LDR threshold calibrated values
  Serial.print("Right average light ");
  Serial.println(rightThresh);
  Serial.print("Mid average light ");
  Serial.println(midThresh);
  Serial.print("Left average light ");
  Serial.println(leftThresh);

  flashLED(1, 1, 1, 1);

  return;
}

#endif

#ifdef DEBUG_LDR

/*function to check state of LDR reeadings against threshold and report if light or dark is seen reletive to refference
   used to check if ambient light levels have changed enough to warrent recalibration*/
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
  return;
}

#endif

/*setspeed for both servos in format that has stop value at 0, forward is + backwars is -
   as opposed to previous format where 90ish was toped and 0 and 180 where full reverse and full forward respectivly*/
void setspeed(int R, int L) {
  rightServo.write(stopR - R);
  leftServo.write(stopL + L);
  return;
}

//stops motors
void Halt() {
  setspeed(0, 0);
  return;
}

/*takes input distance in cm and travels that distance by dividing the desired distance by the circumference of the wheels and multiplying by the time needed for one full rotation,
  setting the servos to move forward and waiting for the desired time to have elapsed */

void Forward(float cm) {
  unsigned int distanceTime = (timeForOneCmRotation * ( cm / wheelCircumference));
  Halt();
  setspeed(rightServoOffset, leftServoOffset);
  delay(distanceTime);
  Halt();
  return;
}

//same but for backwards
void Backward (float cm) {
  unsigned int distanceTime = (timeForOneCmRotation * ( cm / wheelCircumference)); //multiply distance entered by time required for one cm
  Halt();
  setspeed(-rightServoOffset, -leftServoOffset);
  delay(distanceTime);
  Halt();
  return;
}

/*takes input in degrees and multiplies by time taken to turn one degree positive values will turn clockwise, negative values will turn counterclockwise
*/
void turnAngle(int deg) {
  float turnTime = (abs(deg) * timeForOneDegreeTurn); // takes absolute value of input degree incase counterclockwise angle entered, multiplies with time for one degree of turning

#ifdef DEBUG
  Serial.println(deg);
  Serial.println(turnTime);
#endif

  // if angle is positive turn right and delay for calculated amount of time
  if (deg < 0) {
    Halt();
    setspeed(45, -45);
    delay(turnTime);
    Halt();
    return;
  }
  //  if value is negative, turn left and delay for calculated amount of time
  if (deg > 0) {
    Halt();
    setspeed(-45, 45);
    delay(turnTime);
    Halt();
    return;
  }

}


//follows the black line by only moving forward when only the middle LDR can see the black line, takes an input of a desired distance and will follow the line for that distance
void lineFollow(int distanceGoal) {

  unsigned int distanceTravelled = 0;

  //if the distance entered is positive, follows the line forward
  if (distanceGoal > 0) {
    //  goes forwards
    Serial.println("Follow Line Forwards");
    while (distanceTravelled < distanceGoal) {

      setLED(0, 0, 0);
      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* all three don't see line turn 15 degree until line found */
        setLED(0, 1, 0);
        turnAngle(15);
        setLED(0, 0, 0);
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) < rightThresh )) { /* only right sees line hard turn right */
        setLED(0, 1, 0);
        turnAngle(10);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("only right sees line hard turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* right and mid see line gentle turn right */
        setLED(0, 1, 0);
        turnAngle(5);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("right and mid see line gentle turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* best case mid sees line left and right don't on track forwards */
        setLED(0, 1, 0);
        Forward(5);
        distanceTravelled += 3;
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("best case mid sees line left and right don't on track forwards");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* left and mid see line, light turn left */
        setLED(0, 1, 0);
        turnAngle(-5);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("left and mid see line, light turn left");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* only left sees line hard turn left */
        setLED(0, 1, 0);
        turnAngle(-10);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("only left sees line hard turn left ");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* all three see line scan for junction */
#ifdef DEBUG LINE
        Serial.println("all three sensors see black, start scanning for junction and move forward");
#endif
        Forward(1);
        distanceTravelled += 1;
      }
    }
    return;
  }
  //if entered distance is negative, follows the line backwards
  if (distanceGoal < 0) {
    Serial.println("Follow Line Backwards");
    while (distanceTravelled < distanceGoal) {

      setLED(0, 0, 0);
      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* all three don't see line turn 1 degree untill line found */
        turnAngle(15);
        setLED(0, 0, 0);
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) < rightThresh )) { /* only right sees line hard turn right */
        setLED(0, 1, 0);
        turnAngle(10);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("only right sees line hard turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* right and mid see line gentle turn right */
        setLED(0, 1, 0);
        turnAngle(5);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("right and mid see line gentle turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* best case mid sees line left and right don't on track forwards */
        setLED(0, 1, 0);
        Backward(3);
        distanceTravelled += 3;
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("best case mid sees line left and right don't on track forwards");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* left and mid see line, light turn left */
        setLED(0, 1, 0);
        turnAngle(-5);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("left and mid see line, light turn left");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* only left sees line hard turn left */
        setLED(0, 1, 0);
        turnAngle(-10);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("only left sees line hard turn left ");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* all three see line scan for junction */
#ifdef DEBUG_LINE
        Serial.println("all three sensors see black, start scanning for junction and move forward");
#endif
        Backward(1);
        distanceTravelled += 1;
      }
    }
  }
  else {
    return;
  }
}

//same function as used for line following but if a junction has been detected breaks from the function
void junctLineFollow(int distanceGoal) {

  unsigned int distanceTravelled = 0;

  //if the distance entered is positive, follows the line forward
  if (distanceGoal > 0) {
    //  goes forwards
    Serial.println("Follow Line Forwards");
    while (distanceTravelled < distanceGoal) {

      setLED(0, 0, 0);
      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* all three don't see line turn 15 degree until line found */
        setLED(0, 1, 0);
        turnAngle(15);
        setLED(0, 0, 0);
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) < rightThresh )) { /* only right sees line hard turn right */
        setLED(0, 1, 0);
        turnAngle(10);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("only right sees line hard turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* right and mid see line gentle turn right */
        setLED(0, 1, 0);
        turnAngle(5);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("right and mid see line gentle turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* best case mid sees line left and right don't on track forwards */
        setLED(0, 1, 0);
        Forward(5);
        distanceTravelled += 3;
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("best case mid sees line left and right don't on track forwards");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* left and mid see line, light turn left */
        setLED(0, 1, 0);
        turnAngle(-5);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("left and mid see line, light turn left");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* only left sees line hard turn left */
        setLED(0, 1, 0);
        turnAngle(-10);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("only left sees line hard turn left ");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* all three see line scan for junction */
#ifdef DEBUG LINE
        Serial.println("all three sensors see black, start scanning for junction and move forward");
#endif
        return;
      }
    }
    return;
  }
  //if entered distance is negative, follows the line backwards
  if (distanceGoal < 0) {
    Serial.println("Follow Line Backwards");
    while (distanceTravelled < distanceGoal) {

      setLED(0, 0, 0);
      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* all three don't see line turn 1 degree untill line found */
        turnAngle(15);
        setLED(0, 0, 0);
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) < rightThresh )) { /* only right sees line hard turn right */
        setLED(0, 1, 0);
        turnAngle(10);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("only right sees line hard turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* right and mid see line gentle turn right */
        setLED(0, 1, 0);
        turnAngle(5);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("right and mid see line gentle turn right");
#endif
      }

      if ((analogRead(LDRl) > leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* best case mid sees line left and right don't on track forwards */
        setLED(0, 1, 0);
        Backward(3);
        distanceTravelled += 3;
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("best case mid sees line left and right don't on track forwards");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) > rightThresh )) { /* left and mid see line, light turn left */
        setLED(0, 1, 0);
        turnAngle(-5);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("left and mid see line, light turn left");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) > midThresh ) && (analogRead(LDRr) > rightThresh )) { /* only left sees line hard turn left */
        setLED(0, 1, 0);
        turnAngle(-10);
        setLED(0, 0, 0);
#ifdef DEBUG_LINE
        Serial.println("only left sees line hard turn left ");
#endif
      }

      if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) { /* all three see line scan for junction */
#ifdef DEBUG_LINE
        Serial.println("all three sensors see black, start scanning for junction and move forward");
#endif
        return;
      }
    }
  }
  else {
    return;
  }
}

//takes an input value of the number of junctions to count before exiting the function, follows the line counting junctions untill the desired nuber have been counted
void junctionCounter(int junctNumber) {
  unsigned int junctCount = 0;
  //  while the nmumber of junctions seen is less than the desired number follow the line counting junctions
  while (junctCount < junctNumber) {
    setLED(0, 1, 0);
    junctLineFollow(1); //follows line for 1cm
    setLED(0, 0, 0);
    //    if a junction is detected, moves forward until the junction can no longer be seen and increments the junction count by one
    if ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) {
      //      while the junction can be seen, move forward by 1
      while ((analogRead(LDRl) < leftThresh) && (analogRead(LDRm) < midThresh ) && (analogRead(LDRr) < rightThresh )) {
        setLED(1, 0, 0);
        Forward(1);
        setLED(0, 0, 0);

      }
      junctCount++;
    }
    else {

    }
  }
  for (int i = 0; i < junctNumber; i++) {
    setLED(1, 1, 1);
    delay(500);
    setLED(0, 0, 0);
  }
  return;
}

//takes an input time in seconds and struts its stuff while flashing lights for that amount of time, each set of dance moves takes 10 seconds
void danceTime(unsigned int Time) {
  Time = Time * 1000;
  unsigned int start = millis();
  unsigned int endTime = start;
  //  while current duration of loop is smaller than desired duration of loop continue loop
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


void setup() {

  // initiates serial port communicate at baud
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

  //only runs calibration routine if CAL is defined
#ifdef CAL
  calServo();
  Halt();
  setLED(0, 1, 1);
  calLDR();
#endif

#ifdef DEBUG
  //prints EEPROM values for debugging purposes
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

  //  demonstration of straight line movement accuracy
  Forward(50);
  Backward(50);
  //demonstraction of turn in place accurace
  turnAngle(90);
  turnAngle(-90);
  //Dance time
  danceTime(20);

#endif



#ifdef LINE

  //if right pushbutton pressed, simply follow line for 50cm, turn 180, follow for 50cm and follow line backwards for 50cm
  if (checkKEY(0, 1)) {
    Serial.println("follow the line");
    //current values of 75cm are definitly due to battery voltage replace with fresh by friday
    lineFollow(50);
    turnAngle(180);
    lineFollow(50);
    lineFollow(-50);
  }
  else {
  }

#endif


#ifdef OBSTACLE

  //if left pushbutton pressed, follows line forward 1cm at a time scanning for objects every 1cm if an object is detected in the robots path, turn 180 and continue following line
  if (checkKEY(1, 0)) {
    while (true) {
      //      if scan() function returns 1, object is detected, if returns 0 no object
      if (scan()) {
        flashLED(1, 1, 1, 1);
        turnAngle(180);
      }
      else {
        lineFollow(1);
      }
    }
  }
  else {
  }

#endif

#ifdef JUNCT

  //if left and right pushbuttons pressed, count 3 junctions, turn 180 and count 3 junctions
  if (checkKEY(1, 1)) {
    junctionCounter(3);
    turnAngle(180);
    junctionCounter(3);
    Halt();
  }
  else {
  }

#endif
}
