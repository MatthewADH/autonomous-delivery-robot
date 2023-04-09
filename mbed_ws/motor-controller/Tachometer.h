#ifndef _MOTOR_CONTROLLER_TACHOMETER_H_
#define _MOTOR_CONTROLLER_TACHOMETER_H_

#include <mbed.h>


class Tachometer {
public:
  Tachometer(PinName pin, float hz2mps)
        : encoder(pin), HZ2MPS(hz2mps) {
        
    encoder.rise(Callback<void()>(this, &Tachometer::encoderCallback));
    ticker.attach(Callback<void()>(this, &Tachometer::tickerCallback), UPDATE_PERIOD);
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


private:
    InterruptIn encoder;
    Ticker ticker;

    constexpr static float UPDATE_PERIOD = 1 / 50.0;

    const float HZ2MPS;

    int count = 0;
    int numZeros = 0;
    float frequency;


    void encoderCallback() {
        count ++;
    };

    void tickerCallback() {
        frequency = count / UPDATE_PERIOD;
        count = 0;
    };
};

#endif  // _MOTOR_CONTROLLER_TACHOMETER_H_
