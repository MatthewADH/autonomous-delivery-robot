#ifndef _MOTOR_CONTROLLER_TACHOMETER_H_
#define _MOTOR_CONTROLLER_TACHOMETER_H_

#include <mbed.h>


class Tachometer {
public:
  Tachometer(PinName pin, float hz2mps, float update_period)
        : encoder(pin), HZ2MPS(hz2mps), UPDATE_PERIOD(update_period) {
        
    encoder.rise(Callback<void()>(this, &Tachometer::encoderCallback));
  };

  float getFrequency() {
    return frequency;
  };

  float getSpeed() {
    return frequency * HZ2MPS;
  }

  operator float() {
    return getSpeed();
  }

    void update() {
        frequency = count / UPDATE_PERIOD;
        count = 0;
    };


private:
    InterruptIn encoder;

    const float HZ2MPS;
    const float UPDATE_PERIOD;

    int count = 0;
    float frequency;


    void encoderCallback() {
        count ++;
    };

};

#endif  // _MOTOR_CONTROLLER_TACHOMETER_H_
