#ifndef HCSR04_H
#define HCSR04_H

#include "mbed.h"

class Ultrasonic {
    public:

        Ultrasonic(uint8_t trigPin, uint8_t echoPin, /*float period, */float thresh, void warn(void), Timer* t);
        void setStart();
        void checkDistance();
        void trigger(void);

    private:
        const static float inScale_ = 148.0;
        const static float cmScale_ = 58.0;

        float distance_;
        float thresh_;
        Timer* t_;
        Ticker ticker_;    
        uint8_t trig_;
        uint8_t echo_;
        uint32_t start_;

        void (*warn_)(void);
};
#endif