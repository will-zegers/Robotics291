#include "Ultrasonic.h"
#include "TimerOne.h"

#define red_trig_pin  13
#define red_echo_pin  12
#define blue_trig_pin 11
#define blue_echo_pin 10
#define PERIOD        100000
#define THRESHOLD     6.0 //inches

Ultrasonic red(red_trig_pin, red_echo_pin);
Ultrasonic blue(blue_trig_pin, blue_echo_pin);

void setup() {
  Serial.begin(115200);
}

void loop() {
}

void serialEvent() {
  char res;

  res = 'n';
  if(Serial.available() ) {
    Serial.read();
    if(red.check_distance() < THRESHOLD) {
      res = 'r';
    }
    if(blue.check_distance() < THRESHOLD) {
      res = 'b';
    }
    Serial.write(res);
    Serial.flush();
  }
}

