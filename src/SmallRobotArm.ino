#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ServoClass.hpp"
// #include "ServoClassChatGPT.hpp"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define DEFAULT_INTERVAL 50
#define MICROSERVOMIN 100 // This is the 'minimum' pulse length count (out of 4096)
#define MICROSERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN 100
#define SERVOMAX 490
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PI 3.14159

double armOneLength = 102;
double armTwoLength = 77.5;

ServoClass *rotator;
int rotatorStartPos = 90;
bool rotatorFlag = false;
ServoClass *shoulder;
int shoulderStartPos = 0;
bool shoulderFlag = false;
ServoClass *elbow;
int elbowStartPos = 180;
bool elbowFlag = false;
ServoClass *wrist;
int wristStartPos = 170; // TODO: should eventually be 0 when inverses are set
bool wristFlag = false;

int joyXPin = A0;
int joyYPin = A1;

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
  elbow = new ServoClass(2, 490, 100, false, &pwm);
  wrist = new ServoClass(3, 490, 100, false, &pwm);
  startup();

  delay(10);
}

int getPulseWidth(int degrees)
{
  return map(degrees, 0, 180, SERVOMIN, SERVOMAX);
}

int supplementaryPulseWidth(int degrees)
{
  /* Get the "inverse" angle for a servo (ex: 45deg -> 135deg) */
  return map((180 - degrees), 0, 180, SERVOMIN, SERVOMAX);
}

int supplementaryPulseValue(int pulse)
{
  /* Use in a loop when you have mixed-direction servos */
  return SERVOMAX - pulse + SERVOMIN;
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
  int resetSpeed = 50;
  rotator->setMovement(rotatorStartPos, resetSpeed);
  shoulder->setMovement(shoulderStartPos, resetSpeed);
  elbow->setMovement(elbowStartPos, resetSpeed);
  wrist->setMovement(wristStartPos, resetSpeed);
}

void moveToCoords(double x, double y, double z)
{
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(", y: ");
  Serial.print(y);
  Serial.print(", z: ");
  Serial.println(z);

  double baseAngle = atan(x/y) * (180 / PI);
  double armDistance = sqrt(sq(x) + sq(y)); // pythagoras theorem
  Serial.print("armDistance: ");
  Serial.println(armDistance);

  double sideH = sqrt(sq(z) + sq(armDistance));
  Serial.print("sideH: ");
  Serial.println(sideH);
  // Law of Cosines
  double armOneAngle = (sq(armOneLength) + sq(sideH) - sq(armTwoLength)) / (2 * armOneLength * sideH);
  armOneAngle = acos(armOneAngle) * (180 / PI);
  double phi = atan(z/armDistance) * (180 / PI);
  armOneAngle += phi;
  double armTwoAngle = (sq(armOneLength) + sq(armTwoLength) - sq(sideH)) / (2 * armOneLength * armTwoLength);
  armTwoAngle = acos(armTwoAngle) * (180 / PI);
  // rotator->setMovement(baseAngle, 100);
  // rotator->setMoveWithDuration(baseAngle, 1000);
  rotator->directDrive(baseAngle);
  rotatorFlag = true;
  // shoulder->setMovement(armOneAngle, 50);
  // shoulder->setMoveWithDuration(armOneAngle, 1000);
  shoulder->directDrive(armOneAngle);
  shoulderFlag = true;
  // elbow->setMovement(armTwoAngle, 50);
  // elbow->setMoveWithDuration(armTwoAngle, 1000);
  elbow->directDrive(armTwoAngle);
  elbowFlag = true;
  Serial.print("Base angle: ");
  Serial.print(baseAngle);
  Serial.print(", Arm1 angle: ");
  Serial.print(armOneAngle);
  Serial.print(", Arm2 angle: ");
  Serial.println(armTwoAngle);
}

void serialCommands()
{
  // TODO: Inverse trigonometry time baby

  if (Serial.available() > 0)
  {
    String instruction = Serial.readStringUntil('\n');
    Serial.println(instruction);
    char armPiece = instruction[0];
    Serial.println(armPiece);
    int requestedDeg = instruction.substring(1, 4).toInt();
    Serial.println(requestedDeg);
    // TODO: also parse for a given interval? idk man prob not neccessary
    switch (armPiece)
    {
    case 'r':
      // rotator->setMovement(requestedDeg, DEFAULT_INTERVAL);
      rotator->setMoveWithDuration(requestedDeg, 1000);
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
      moveToCoords(20, 0, 10);
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
  int joyX = analogRead(joyXPin);
  int joyY = analogRead(joyYPin);
  // Serial.print(joyX);
  // Serial.print(", ");
  // Serial.println(joyY);
  double mappedX = map(joyX, 1, 1024, 0, 100);
  double mappedY = map(joyY, 1, 1024, 0, 100);
  // Serial.print("X: ");
  // Serial.print(mappedX);
  // Serial.print(", Y: ");
  // Serial.println(mappedY);
  moveToCoords(50, 50, 20);
  // serialCommands();
  // if (Serial.available() > 0)
  // {
  //   String instruction = Serial.readStringUntil('\n');

  //   moveToCoords(25, 0, instruction.toInt());
  // }
  // if (rotatorFlag)
  //   // rotator->update();
  //   rotator->updateWithDuration();
  // if (shoulderFlag)
  //   shoulder->updateWithDuration();
  // if (elbowFlag)
  //   elbow->updateWithDuration();
  // if (wristFlag)
  //   wrist->updateWithDuration();

  // for(int servo=0; servo<4; servo++) {
  //     Serial.println(servo);
  //     pwm.setPWM(servo, 0, pulselength);
  //     delay(100);
  // }
  // TODO: Look at adafruit's millis guide
  // for (int pulse = getPulseWidth(60); pulse < getPulseWidth(120); pulse++) {
  //   Serial.println(pulse);
  //   pwm.setPWM(0, 0, pulse);
  //   pwm.setPWM(1, 0, supplementaryPulseValue(pulse));
  //   pwm.setPWM(3, 0, pulse);
  //   pwm.setPWM(2, 0, supplementaryPulseValue(pulse));
  //   delay(30);
  // }
  // for (int pulse = getPulseWidth(120); pulse > getPulseWidth(60); pulse--) {
  //   pwm.setPWM(0, 0, pulse);
  //   pwm.setPWM(1, 0, supplementaryPulseValue(pulse));
  //   pwm.setPWM(3, 0, pulse);
  //   pwm.setPWM(2, 0, supplementaryPulseValue(pulse));
  //   delay(30);
  // }
}
