#include "HCSR04.h"

//Ultrasonic* Ultrasonic::instance = 0;
Ultrasonic::Ultrasonic(uint8_t trigPin, uint8_t echoPin, /*float period, */float thresh, void warn(void), Timer* t) {

//    instance = this;

    trig_ = trigPin;
    echo_ = echoPin;
    LPC_GPIO2->FIODIR      |=  (1 << trig_);
    LPC_GPIO2->FIODIR      &= ~(1 << echo_);
  
    LPC_GPIOINT->IO2IntEnR |=  (1 << echo_);
    LPC_GPIOINT->IO2IntEnF |=  (1 << echo_);
    
    thresh_ = thresh;
    distance_ = thresh_ + 1;
    
    warn_ = warn;
 //   ticker_.attach(&isr, period);
    t_ = t;
}

void Ultrasonic::setStart() {
    start_ = (*t_).read_us();   
}

void Ultrasonic::checkDistance() {
    distance_ = ((*t_).read_us() - start_) / inScale_;
    if(distance_ < thresh_) {
        warn_();   
    }    
}

void Ultrasonic::trigger(void) {

    LPC_GPIO2->FIOCLR |= (1 << trig_);
    wait_us(2);
    LPC_GPIO2->FIOSET |= (1 << trig_);
    wait_us(10);
}