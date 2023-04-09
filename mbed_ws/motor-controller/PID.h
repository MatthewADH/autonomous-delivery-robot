#ifndef _MOTOR_CONTROLLER_pid_h
#define _MOTOR_CONTROLLER_pid_h

#include <mbed.h>
#include <math.h>

#include "Tachometer.h"

class PID {
public:
  PID(PinName feedbackPin, float hz2mps, PinName outputPin, float kp,
        float ki, float kd, float sampleTime) :
        feedback(feedbackPin, hz2mps), output(outputPin), kp(kp), ki(ki),
        kd(kd), T(sampleTime) {
    
    output.period(T);
  };

  void setKp(float val) {
    kp = val;
  }

  void setKi(float val) {
    ki = val;
  }

  void setKd(float val) {
    kd = val;
  }

  void setGains(float p, float i, float d) {
    setKp(p);
    setKi(i);
    setKd(d);
  }

  void setReference(const float& ref) {
    reference = ref;
  }

  void operator = (const float& ref) {
    setReference(ref);
  }

  void step() {
    error = reference - feedback;
    rate = (error - prevError) / T;
    if (fequal(controlSignal, output.read()) || error * controlSignal < 0)
      area = prevArea + error * T;

    controlSignal  = kp * error + ki * area + kd * rate;
    output = clamp(controlSignal, MIN_DUTY, MAX_DUTY);

    prevError = error;
    prevArea = area;
  };

private:
  constexpr static float MIN_DUTY = 0.0;
  constexpr static float MAX_DUTY = 1.0;

  Tachometer feedback;
  PwmOut output;  // Motor voltage control signal via varying duty cycle 
  
  float kp, ki, kd;
  const float T;  // sample time

  float reference;
  float error;
  float prevError;
  float area;
  float prevArea;
  float rate;
  float controlSignal;
  

  template<class T>
  T clamp(const T& value, const T& min, const T& max)  {
    if (value < min)
      return min;
    else if (value > max)
      return max;
    else
      return value;
  };

  bool fequal (float x, float y, float eps = 0.0001) {
    return fabs(x - y) < eps;
  };

};

#endif  // _MOTOR_CONTROLLER_pid_h__
