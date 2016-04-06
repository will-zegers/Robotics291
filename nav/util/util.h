#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#ifndef util_h
#define util_h

#define SENSOR_WAIT 25000

#define SPEED_U     0
#define SPEED_L     150
#define STEP	    150
#define BUFFSIZE    256
#define UART2       "/dev/ttyO2"
#define UART4       "/dev/ttyO4"
#define SENSOR_WAIT 25000

	int uart2, uart4;
	short x, y;
	const char runScript	   = {153};

	short orient();
	short getAngle();
	void step(short stepSize);
	void turnAngle(int angle); 
	void rotateBasis(short theta);
	short arctan(double x, double y);
	int initSerial(const char* path);
	int getSensorDistance(char* detected);
#endif
