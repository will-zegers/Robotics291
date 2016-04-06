#include "ball.h"

// Default Constructor
Ball::Ball()
{
	//set values for default constructor
	setColor("green"); //Todo, put to null
	xPos = -1;
	yPos = -1;
	depth = -1;
	area = -1;
	tag = -1;
}

// Destructor
Ball::~Ball(void)
{
}


int Ball::getXPos() {
	return xPos;
}

void Ball::setXPos(int x) {
	xPos = x;
}

int Ball::getYPos() {
	return yPos;
}

void Ball::setYPos(int y) {
	yPos = y;
}

int Ball::getDepth() {
	return depth;
}

void Ball::setDepth(int z) {
	depth = z;
}

double Ball::getArea() {
	return area;
}

void Ball::setArea(double a) {
	area = a;
}

int Ball::getTag() {
	return tag;
}

void Ball::setTag(int t) {
	tag = t;
}