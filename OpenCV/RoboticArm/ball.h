#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>

using namespace std;

class Ball
{
public:
	Ball();
	~Ball(void);

	int getXPos();
	void setXPos(int x);

	int getYPos();
	void setYPos(int y);

	int getDepth();
	void setDepth(int z);

	double getArea();
	void setArea(double a);

	string getColour(){
		return color;
	}
	void setColor(string c){
		color = c;
	}

private:

	int xPos, yPos, depth;
	double area;
	string color;

};