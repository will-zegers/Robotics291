#include "NavServer.h"

NavServer::Navserver(const char* uart2, const char* uart4) {
	uart2_ = initSerial(uart2);
	write(uart2_,
}

short NavServer::orient() {
	short angle;

    angle = arctan(x,y);

    turnAngle(angle);

    rotateBasis(-angle);

    return angle;
}

short NavServer::getAngle() {
	
	const char readAngleSensor[2] = {142,20};
    char recvBuff[2];

    write(uart2,&readAngleSensor,2);
    usleep(SENSOR_WAIT);
    read(uart2,&recvBuff,2);

    return -((recvBuff[0] << 8) | recvBuff[1]);
}

void NavServer::step(short stepSize) {

	char forward[15]        = {152,13,137,SPEED_U,SPEED_L,128,0,156,0,STEP,137,0,0,0,0};
    forward[8] = stepSize >> 8;
    forward[9] = stepSize & 0xff;

    write(uart2,&forward,15);
    write(uart2,&runScript,1);

    usleep(STEP*1000000/SPEED_L);
}

void NavServer::turnAngle(int angle) {
	char turn[15] = {152,13,137,SPEED_U,SPEED_L,0,1,157,0,0,137,0,0,0,0};

    turn[8] = angle >> 8;
    turn[9] = angle & 0xff;

    if(angle < 0) {
        turn[5] = 255;
        turn[6] = 255;
    }

    write(uart2,&turn,15);
    write(uart2,&runScript,1);
    usleep(33333 * abs(angle));
}

void NavServer::rotateBasis(short theata) {
	short temp_x,temp_y;

    temp_x = cos(theta*M_PI/180) * (x) + -sin(theta*M_PI/180) * (y);
    temp_y = sin(theta*M_PI/180) * (x) + cos(theta*M_PI/180) * (y);

    x = temp_x;
    y = temp_y;
}

short NavServer::arctan(double x, double y) {
	short theta;

    if(x == 0)
        return (y > 0) ? 90 : -90;

    theta = ceil(atan(y/x)*180/M_PI);
    if(x < 0) {
        return (y < 0) ? theta - 180 : theta + 180;
    } else {
        return theta;
    }
}

int NavServer::initSerial(const char* path) {
	fd = open(path, O_RDWR | O_NOCTTY);

    if(fd < 0) {
        printDBG(stderr, "Port failed_on to open\n");
        exit(0);
    }

    tcgetattr(fd, &old);
    memset(&uart, 0, sizeof uart);

    if(strcmp(path,UART2) == 0)
        uart.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
    else
        uart.c_cflag = B115200 | CS8 | CLOCAL | CREAD;

    uart.c_iflag = IGNPAR | ICRNL;
    uart.c_oflag = 0;
    uart.c_lflag = 0;

    uart.c_cc[VTIME] = 0;
    uart.c_cc[VMIN]  = 1;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&uart);

    return fd;
}
