#include "mbed.h"
#include "HCSR04.h"

#define LED1  18
#define LED2  20
#define LED4  23
#define R_TRIG 5
#define R_ECHO 4
#define B_TRIG 3
#define B_ECHO 2
#define PERIOD 0.2

void irq(void);
void warnRed(void);
void initGpio(void);
void heartbeat(void);
void warnBlue(void);
void initEINT3(void);

Serial usb(USBTX,USBRX);

//const int red = 
Timer t;
Ticker tick1, tick2;

Ultrasonic blue(B_TRIG, B_ECHO, 6.0, &warnBlue, &t);
Ultrasonic red(R_TRIG, R_ECHO, 6.0, &warnRed, &t);

int main() {
    usb.baud(115200);
    initGpio();
    initEINT3();
    
    t.start();
    tick1.attach(&red, &Ultrasonic::trigger, PERIOD);
    tick2.attach(&blue, &Ultrasonic::trigger, PERIOD);
    
    for(;;) {
        heartbeat();
        heartbeat();
        wait_ms(800);
    }
}

void heartbeat(void) {
    LPC_GPIO1->FIOSET = (1 << LED2);
    wait_ms(100);
    LPC_GPIO1->FIOCLR = (1 << LED2);
    wait_ms(200);    
}

void initGpio(void) {
    LPC_GPIO1->FIODIR |= (1 << LED1);
    LPC_GPIO1->FIODIR |= (1 << LED2);
    LPC_GPIO1->FIODIR |= (1 << LED4);
    
    LPC_GPIO1->FIOCLR = (1 << LED1);
    LPC_GPIO1->FIOCLR = (1 << LED2);
    LPC_GPIO1->FIOCLR = (1 << LED4);
}

void irq(void) {
    if (LPC_GPIOINT->IO2IntStatR & (1 << R_ECHO)) {
        LPC_GPIOINT->IO2IntClr |= (1 << R_ECHO);
        red.setStart();
    } else if (LPC_GPIOINT->IO2IntStatF & (1 << R_ECHO)) {
        LPC_GPIOINT->IO2IntClr |= (1 << R_ECHO);
        red.checkDistance();
    } else if (LPC_GPIOINT->IO2IntStatR & (1 << B_ECHO)) {
        LPC_GPIOINT->IO2IntClr |= (1 << B_ECHO);
        blue.setStart();
    } else if (LPC_GPIOINT->IO2IntStatF & (1 << B_ECHO)) {
        LPC_GPIOINT->IO2IntClr |= (1 << B_ECHO);
        blue.checkDistance();
    }    
}

void initEINT3() {
    NVIC_SetVector(EINT3_IRQn, (uint32_t)&irq);
    NVIC_EnableIRQ(EINT3_IRQn);
}

void warnRed(void) {
    usb.putc('r');
}

void warnBlue(void) {
    usb.putc('b');
}
