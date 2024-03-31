#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class PWMServoWrapper
{
private:
  Adafruit_PWMServoDriver *pwm; // pwm object
  int minPulse, maxPulse;       // min and max pwm pulse for the servo
  int pin;                      // pin number on the driver board
  bool inverse;                 // invert the servo: ex. 45deg into 135deg
  int pos = 0;                  // current servo position
  int increment = 1;            // increment to move for each interval
  int updateInterval = 0;       // interval between updates
  unsigned long lastUpdate;     // last update of position
  int targetDeg;                // target degree
  int timedDuration = 0;        // duration to hit target
  unsigned long startTime;      // duration start time

  /* Get the pulse width needed to drive the servo */
  int getPulseWidth(int degrees) { return map(degrees, 0, 180, minPulse, maxPulse); }

  /* Get the "inverse" pulse for a servo (ex: 45deg -> 135deg) */
  int supplementaryPulseWidth(int degrees) { return map((180 - degrees), 0, 180, minPulse, maxPulse); }

  /* Update function for duration-based movement */
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
      timedDuration = 0; // reset duration for update checking
    }
  }

  /* Update function for interval-based movement */
  void updateWithInterval()
  {
    if ((millis() - lastUpdate) > updateInterval) // time to update
    {
      if (increment) // don't send any more pulses if increment is 0
      {
        lastUpdate = millis();
        pos += increment;
        double pulse = inverse ? supplementaryPulseWidth(pos) : getPulseWidth(pos);
        pwm->setPWM(pin, 0, pulse);
        if (increment < 0)
        {
          if ((pos <= targetDeg))
          {
            increment = 0;
            updateInterval = 0; // reset interval for update checking
          }
        }
        else
        {
          if ((pos >= targetDeg))
          {
            increment = 0;
            updateInterval = 0;
          }
        }
      }
    }
  }

public:
  PWMServoWrapper(int pin, int maxPulse, int minPulse, bool inverse, Adafruit_PWMServoDriver *pwm)
  {
    this->pwm = pwm;
    this->pin = pin;
    this->maxPulse = maxPulse;
    this->minPulse = minPulse;
    this->inverse = inverse;
  }

  void detach() { pwm->setPWM(pin, 0, 4096); }   // effectively detaches servo and will drop power
  void setStartPos(int degree) { pos = degree; } // used for start pos or for robot startup
  int getPos() { return pos; }                   // return last position

  /* Drive motor directly to a position. Only call every loop if you're slowly increasing deg */
  void directDrive(int degree)
  {
    double pulse = inverse ? supplementaryPulseWidth(degree) : getPulseWidth(degree);
    pwm->setPWM(pin, 0, pulse);
  }

  /* Set a target degree and "speed" by using a step interval */
  void setMovement(int degree, int stepInterval)
  {
    targetDeg = degree;
    updateInterval = stepInterval;
    increment = 1; // reset back to 1 after update() finishes and sets to 0
    if (targetDeg < pos)
    {
      increment = -1;
    }
  }

  /* Set a target degree and duration. Moves to the target within that duration*/
  void setMoveWithDuration(int targetDegree, int durationMillis)
  {
    startTime = millis();
    targetDeg = targetDegree;
    timedDuration = durationMillis;
  }

  /* Call to incrementally update position - use in loop() without a delay */
  void update()
  {
    if (updateInterval > 0)
    {
      updateWithInterval();
    }
    else if (timedDuration > 0)
    {
      updateWithDuration();
    }
  }
};