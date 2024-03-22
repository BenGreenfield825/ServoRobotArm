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

// our servo # counter
uint8_t servonum = 0;

// ServoClass *shoulder;
ServoClass *rotator;
int rotatorStartPos = 90;
bool rotatorFlag = false;
ServoClass *shoulder;
int shoulderStartPos = 0;
bool shoulderFlag = false;
ServoClass *elbow;
int elbowStartPos = 0;
bool elbowFlag = false;
ServoClass *wrist;
int wristStartPos = 170; // TODO: should eventually be 0 when inverses are set
bool wristFlag = false;

void setup()
{
  Serial.begin(9600);
  Serial.println("Robot arm starting...");

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
    // TODO: also parse for a given interval? idk man prob not neccessary
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
  serialCommands();

  if (rotatorFlag)
    rotator->update();
  if (shoulderFlag)
    shoulder->update();
  if (elbowFlag)
    elbow->update();
  if (wristFlag)
    wrist->update();

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
