#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class ServoClass
{
private:
  Adafruit_PWMServoDriver *pwm;
  int maxPulse, minPulse;
  int pin;
  bool inverse;
  bool flippedRange;
  int pos = 0;              // current servo position
  int increment = 1;        // increment to move for each interval
  int updateInterval = 30;  // interval between updates
  unsigned long lastUpdate; // last update of position
  int targetDeg;
  int timedDuration;
  unsigned long startTime;

  int getPulseWidth(int degrees)
  {
    return map(degrees, 0, 180, minPulse, maxPulse);
  }

  int supplementaryPulseWidth(int degrees)
  {
    /* Get the "inverse" angle for a servo (ex: 45deg -> 135deg) */
    return map((180 - degrees), 0, 180, minPulse, maxPulse);
  }

  // int supplementaryPulseValue(int pulse)
  // {
  //   /* Use in a loop when you have mixed-direction servos */
  //   return maxPulse - pulse + minPulse;
  // }

public:
  ServoClass(int pin, int maxPulse, int minPulse, bool inverse, Adafruit_PWMServoDriver *pwm)
  {
    this->pwm = pwm;
    this->pin = pin;
    this->maxPulse = maxPulse;
    this->minPulse = minPulse;
    this->inverse = inverse;
  }

  void detach() { pwm->setPWM(pin, 0, 4096); } // effectively detaches servo and will drop power

  void setStartPos(int degree) { pos = degree; } // used for start pos or for robot startup
  int getPos() { return pos; }                   // return last position

  void directDrive(int degree) {
    /// Drive motor directly to a position
    double pulse = inverse ? supplementaryPulseWidth(degree) : getPulseWidth(degree);
    pwm->setPWM(pin, 0, pulse);
  }

  void setMovement(int degree, int stepInterval)
  {
    targetDeg = degree;
    updateInterval = stepInterval;
    increment = 1; // reset this back to 1 after update() finishes and sets to 0
    if (targetDeg < pos)
    {
      increment = -1;
    }
  }

  void setMoveWithDuration(int targetDegree, int durationMillis)
  {
    startTime = millis();           // Record the start time
    targetDeg = targetDegree;       // Set the target position
    timedDuration = durationMillis; // Set the duration for movement
  }

  void updateWithDuration()
  {
    unsigned long elapsedTime = millis() - startTime; // Calculate elapsed time

    if (elapsedTime <= timedDuration) // Check if movement duration hasn't elapsed
    {
      // Calculate the position based on elapsed time and target position
      int currentPos = map(elapsedTime, 0, timedDuration, pos, targetDeg + 1);
      double pulse = inverse ? supplementaryPulseWidth(currentPos) : getPulseWidth(currentPos);
      pwm->setPWM(pin, 0, pulse);
    }
    else // Movement duration has elapsed
    {
      // Move to the target position directly (doesn't usually hit the target exactly during duration)
      double pulse = inverse ? supplementaryPulseWidth(targetDeg) : getPulseWidth(targetDeg);
      pwm->setPWM(pin, 0, pulse);
      pos = targetDeg;
    }
  }

  void update()
  /// Call to incrementally update position - use in loop() without a delay
  {
    if ((millis() - lastUpdate) > updateInterval) // time to update
    {
      if (increment) // don't send any more pulses if increment is 0
      {
        lastUpdate = millis();
        pos += increment;
        double pulse = inverse ? supplementaryPulseWidth(pos) : getPulseWidth(pos);
        pwm->setPWM(pin, 0, pulse);
        // Serial.println(pos);
        if (increment < 0)
        {
          if ((pos <= targetDeg))
          {
            increment = 0;
          }
        }
        else
        {
          if ((pos >= targetDeg))
          {
            increment = 0;
          }
        }
      }
    }
  }
};