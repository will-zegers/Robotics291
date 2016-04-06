#ifndef Ultrasonic_h
#define Ultrasonic_h

#include "Arduino.h"
#include "stdint.h"

class Ultrasonic {
  public:
    
    Ultrasonic(uint8_t, uint8_t);

    float check_distance(void);
  
  private:
    const uint32_t inScale_ = 148;
    const uint32_t cmScale_ = 58;
  
    uint32_t thresh_;
    uint8_t trig_;
    uint8_t echo_;
};

#endif
