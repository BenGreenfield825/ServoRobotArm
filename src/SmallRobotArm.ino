#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ServoClass.hpp"

#define DEFAULT_INTERVAL 50
#define MICROSERVOMIN 100 // This is the 'minimum' pulse length count (out of 4096)
#define MICROSERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN 100
#define SERVOMAX 490
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PI 3.14159
#define RADIANS_OFF 180/PI
#define DEBUG true

double armOneLength = 102;
double armTwoLength = 77.5;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
ServoClass *rotator;
int rotatorStartPos = 90;
bool rotatorFlag = false;
ServoClass *shoulder;
int shoulderStartPos = 115;
bool shoulderFlag = false;
ServoClass *elbow;
int elbowStartPos = 50;
bool elbowFlag = false;
ServoClass *wrist;
int wristStartPos = 180;
bool wristFlag = false;

int joyXPin = A0;
int joyYPin = A1;

long lastUpdate;
int updateInterval = 2000;
bool forward = true;

void setup()
{
  Serial.begin(9600);
  Serial.println("Robot arm starting...");

  pinMode(joyXPin, INPUT);
  pinMode(joyYPin, INPUT);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  rotator = new ServoClass(0, 490, 100, false, &pwm);
  shoulder = new ServoClass(1, 490, 100, false, &pwm);
  elbow = new ServoClass(2, 490, 100, true, &pwm);
  wrist = new ServoClass(3, 490, 100, false, &pwm);
  startup();

  delay(10);
}

void startup()
{
  // TODO: maybe make them move to start position
  rotator->setStartPos(rotatorStartPos);
  shoulder->setStartPos(shoulderStartPos);
  elbow->setStartPos(elbowStartPos);
  wrist->setStartPos(wristStartPos);
}

void resetPosition()
{
  // TODO: switch to timed movement
  int resetSpeed = 50;
  rotator->setMovement(rotatorStartPos, resetSpeed);
  rotatorFlag = true;
  shoulder->setMovement(shoulderStartPos, resetSpeed);
  shoulderFlag = true;
  elbow->setMovement(elbowStartPos, resetSpeed);
  elbowFlag = true;
  wrist->setMovement(wristStartPos, resetSpeed);
  wristFlag = true;
}

void moveToCoords(double x, double y, double z)
{
  // TODO: add hard limits based on physical restrictions
  double baseAngle = atan(x / y) * RADIANS_OFF; // angle to rotate the base
  double armDistance = sqrt(sq(x) + sq(y));    // pythagoras theorem - how far out the arm reaches in x/y

  double h = sqrt(sq(z) + sq(armDistance)); // hypotenuse for the first two arm links

  // Law of Cosines - find the missing angles using side lengths
  double armOneAngle = (sq(armOneLength) + sq(h) - sq(armTwoLength)) / (2 * armOneLength * h);
  armOneAngle = acos(armOneAngle) * RADIANS_OFF;
  double phi = atan(z / armDistance) * RADIANS_OFF;
  armOneAngle += phi;
  double armTwoAngle = (sq(armOneLength) + sq(armTwoLength) - sq(h)) / (2 * armOneLength * armTwoLength);
  armTwoAngle = acos(armTwoAngle) * RADIANS_OFF;

  // Calculations to keep the wrist parallel to the ground - see reference photo for an example
  // There is probably a better way to do this, but it works and are simple calculations
  double a3 = 180 - (armOneAngle - phi) - armTwoAngle; // third angle - 180deg minus our other angles
  double a4 = 180 - phi - 90; 
  double a5 = 180 - a4 - a3;
  double a6 = a5 - 90;
  double a7 = 180 - a6 - 90; // include 90deg offset
  if (a7 > 180)
    a7 -= 360; // our servo can't do more than 180deg, so subtract 360 to "wrap back around"

  rotator->setMoveWithDuration(baseAngle, 1000);
  rotatorFlag = true;
  shoulder->setMoveWithDuration(armOneAngle, 1000);
  shoulderFlag = true;
  elbow->setMoveWithDuration(armTwoAngle, 1000);
  elbowFlag = true;
  wrist->setMoveWithDuration(a7, 1000);
  wristFlag = true;

  if (DEBUG)
  {
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.print(y);
    Serial.print(", z: ");
    Serial.println(z);
    Serial.print("Base angle: ");
    Serial.print(baseAngle);
    Serial.print(", phi: ");
    Serial.print(phi);
    Serial.print(", Arm1 angle: ");
    Serial.print(armOneAngle);
    Serial.print(", Arm2 angle: ");
    Serial.println(armTwoAngle);
  }
}

void inverseKinematicsDemo()
{
  if ((millis() - lastUpdate) > updateInterval)
  {
    lastUpdate = millis();
    if (forward)
    {
      moveToCoords(0, 0, 150);
      forward = false;
    }
    else
    {
      moveToCoords(0, 0, 80);
      forward = true;
    }
  }
}

void serialCommands()
{
  if (Serial.available() > 0)
  {
    String instruction = Serial.readStringUntil('\n');
    Serial.println(instruction);
    char armPiece = instruction[0];
    Serial.println(armPiece);
    int requestedDeg = instruction.substring(1, 4).toInt();
    Serial.println(requestedDeg);
    switch (armPiece)
    {
    case 'r':
      rotator->setMovement(requestedDeg, DEFAULT_INTERVAL);
      rotatorFlag = true;
      break;
    case 's':
      shoulder->setMovement(requestedDeg, DEFAULT_INTERVAL);
      shoulderFlag = true;
      break;
    case 'e':
      elbow->setMovement(requestedDeg, DEFAULT_INTERVAL);
      elbowFlag = true;
      break;
    case 'w':
      wrist->setMovement(requestedDeg, DEFAULT_INTERVAL);
      wristFlag = true;
      break;
    case 'c':
      moveToCoords(60, 40, 80);
      break;
    case 'd':
      inverseKinematicsDemo();
      break;
    case 'x':
      Serial.println("RESET");
      resetPosition();
    // Cases needed for wrist rotation and hand
    default:
      break;
    }
  }
}

void loop()
{
  // int joyX = analogRead(joyXPin);
  // int joyY = analogRead(joyYPin);
  // Serial.print(joyX);
  // Serial.print(", ");
  // Serial.println(joyY);
  // double mappedX = map(joyX, 1, 1024, 0, 100);
  // double mappedY = map(joyY, 1, 1024, 0, 100);
  // Serial.print("X: ");
  // Serial.print(mappedX);
  // Serial.print(", Y: ");
  // Serial.println(mappedY);
  // moveToCoords(60, 40, 70);
  serialCommands();
  // inverseKinematicsDemo();
  // if (Serial.available() > 0)
  // {
  //   String instruction = Serial.readStringUntil('\n');

  //   moveToCoords(60, 40, instruction.toInt());
  // }
  if (rotatorFlag)
    rotator->update();
  if (shoulderFlag)
    shoulder->update();
  if (elbowFlag)
    elbow->update();
  if (wristFlag)
    wrist->update();
}
