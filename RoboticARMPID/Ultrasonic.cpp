#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trig_pin, uint8_t echo_pin) {

  
  trig_ = trig_pin;
  echo_ = echo_pin;
  pinMode(trig_, OUTPUT);
  pinMode(echo_, INPUT);
}

float Ultrasonic::check_distance(void) {
  
  digitalWrite(trig_, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_, HIGH);
  delayMicroseconds(10);

  return (float)((float)pulseIn(echo_, HIGH) / (float)inScale_);
}

