#include "mbed.h"

#define FREQ   38000 // Hz
#define BEACON 4     // Hz
#define D_SIZE 8     // bits

#define IR_ARRAY (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)

Serial pc(USBTX,USBRX);
DigitalOut myled(LED1);

const int data[D_SIZE] = {0,1,0,1,0,1,1,1}; // W

void setup() {
    myled = 1;
    LPC_GPIO2->FIODIR |= IR_ARRAY;
    LPC_GPIO2->FIOSET |= IR_ARRAY;
}

int main() {
    int i, j;
    setup();
    while(1) {
        i = 0;
        while(i < D_SIZE) {
            j = 0;
            if(data[i]) {
                while(j < FREQ/(D_SIZE * BEACON) ) {
                    wait_us(1000000/(2*FREQ) );
                    LPC_GPIO2->FIOCLR = IR_ARRAY;
                    wait_us(1000000/(2*FREQ) );
                    LPC_GPIO2->FIOSET = IR_ARRAY;
                    j++;
                }
            } else {
                while(j < FREQ/(D_SIZE * BEACON) ) {
                    wait_us(1000000/FREQ);
                    j++;
                }
            }
            i++;
        }
    }
}

