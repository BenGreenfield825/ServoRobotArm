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

  void update()
  /// Call to incrementally update position - use in loop() without a delay
  {
    if ((millis() - lastUpdate) > updateInterval) // time to update
    {
      if (increment) // don't send any more pulses if increment is 0
      {
        lastUpdate = millis();
        pos += increment;
        pwm->setPWM(pin, 0, getPulseWidth(pos));
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