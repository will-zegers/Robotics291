#ifndef NAVSERVER_H
#define NAVSERVER_H

class NavServer {

	public:
		NavServer();
	private:
		static int uart2_, uart4_;
		static short x_, y_;
		const char runScript = 153;

		short orient();
		short getAngle();
		void step(short stepSize);
		void turnAngle(int angle);
		void rotateBasis(short theta);
		short acrtan(double x, double y);
		int initSerial(const char* path);
};
#endif

