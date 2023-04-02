#ifndef _MOTOR_CONTROLLER_pid_h
#define _MOTOR_CONTROLLER_pid_h

#include <mbed.h>
#include <math.h>

class PID {
public:
  PID(PinName feedbackPin, PinName outputPin, float kp, float ki, float kd, 
        float sampleTime) :
        feedback(feedbackPin), output(outputPin), KP(kp), KI(ki), KD(kd), 
        T(sampleTime) {
    
    output.period(T);
    reference = 0.5;
  };

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

    controlSignal  = KP * error + KI * area + KD * rate;
    output = clamp(controlSignal, MIN_DUTY, MAX_DUTY);

    prevError = error;
    prevArea = area;
  };

private:
  constexpr static float MIN_DUTY = 0.0;
  constexpr static float MAX_DUTY = 1.0;

  AnalogIn feedback;  // Voltage representing wheel speed from tachometer 
  PwmOut output;  // Motor voltage control signal via varying duty cycle 
  
  const float KP, KI, KD;
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
