#ifndef _MOTOR_CONTROLLER_pid_h
#define _MOTOR_CONTROLLER_pid_h

#include <mbed.h>

#include "Tachometer.h"

class PID {
public:
  PID(PinName feedbackPin, float hz2mps, PinName outputPin, float kp,
            float ki, float kd, float sampleTime, float acceleration) :
        feedback(feedbackPin, hz2mps, sampleTime), output(outputPin), 
        kp(kp), ki(ki), kd(kd), T(sampleTime), ACCELERATION(acceleration),
        VEL_STEP(acceleration * sampleTime) {
    
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

  void setTarget(const float& tar) {
    target = tar;
  }

  void operator = (const float& tar) {
    setTarget(tar);
  }

  operator float () {
    return output.read();
  }

  float read () {
    return output.read();
  }

  float getFeedback() {
    return feedback;
  }

  void step() {
    feedback.update();

    if (target == 0) {
      reference = 0;
    }
    else if (reference != target) {
      if (target - reference > 0) {
        if (reference + VEL_STEP < target)
          reference += VEL_STEP;
        else
          reference = target;
      }
      else {
        if (reference - VEL_STEP > target)
          reference -= VEL_STEP;
        else
          reference = target;
      }
    }

    error = reference - feedback;
    rate = (error - prevError) / T;
    if (fequal(controlSignal, output.read()) || error * controlSignal < 0)
      area = prevArea + error * T;

    controlSignal  = kp * error + ki * area + kd * rate;
    if (reference == 0) output = 0;
    else output = clamp(controlSignal, MIN_DUTY, MAX_DUTY);
    
    prevError = error;
    prevArea = area;
    if (reference == 0 && feedback == 0) prevArea = 0;
  };

private:
  constexpr static float MIN_DUTY = 0.0;
  constexpr static float MAX_DUTY = 0.4;

  Tachometer feedback;
  PwmOut output;  // Motor voltage control signal via varying duty cycle 
  
  float kp, ki, kd;
  const float T;  // sample time
  const float ACCELERATION;
  const float VEL_STEP;

  float target = 0.0;
  float reference = 0.0;
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
    float dif = x - y;
    return dif > -eps && dif < eps;
  };

};

#endif  // _MOTOR_CONTROLLER_pid_h__
