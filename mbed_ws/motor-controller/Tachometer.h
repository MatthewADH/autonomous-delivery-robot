#ifndef _MOTOR_CONTROLLER_TACHOMETER_H_
#define _MOTOR_CONTROLLER_TACHOMETER_H_

#include <limits>
#include <mbed.h>
#include <chrono>


class Tachometer {
public:
    Tachometer(PinName pin)
            : encoder(pin, PullUp) {
        
        encoder.rise(Callback<void()>(this, &Tachometer::encoderCallback));
        ticker.attach(Callback<void()>(this, &Tachometer::tickerCallback), UPDATE_PERIOD);
    };

    float getFrequency() {
        return numSamples * 1e6 / measuredTime_us;
    };


private:
    InterruptIn encoder;
    HighResClock clock;
    Ticker ticker;

    const static int SAMPLE_DURATION_US = 10000;
    const static int MAX_MEASURABLE_PERIOD_US = 500000;
    constexpr static float UPDATE_PERIOD = 3e-6 * SAMPLE_DURATION_US;
    int numSamples = 1;
    int count = 0;
    int measuredTime_us = numeric_limits<int>::max();
    int curTime, prevTime;
    bool reset = false;

    void encoderCallback() {
        count ++;
        if (!reset && count < numSamples) return;

        curTime = clock.now().time_since_epoch().count();
        measuredTime_us = curTime - prevTime;
        numSamples = 1 + SAMPLE_DURATION_US * count / measuredTime_us;
        prevTime = curTime;
        count = 0;
        reset = false;
    };

    void tickerCallback() {
        if (clock.now().time_since_epoch().count() - curTime > MAX_MEASURABLE_PERIOD_US)
            measuredTime_us = numeric_limits<int>::max();
        else if (clock.now().time_since_epoch().count() - curTime > UPDATE_PERIOD)
            reset = true;
    };
};

#endif  // _MOTOR_CONTROLLER_TACHOMETER_H_