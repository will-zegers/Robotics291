#include <cstdlib>
#include <iostream>
#include <highgui.h>
#include <stdio.h>
#include <sstream>
#include <math.h>
#include <limits.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ball.h"

using namespace cv;
using namespace std;
using namespace cv_bridge;

string left_img_title = "Left Image";
string right_img_title = "Right Image";
string thresholded_img_right_title = "Thresholded Image Right";
string thresholded_img_left_title = "Thresholded Image Left";
string trackbarWindowName = "Trackbars";

const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
// max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
// minimum and maximum object area
const int MIN_OBJECT_AREA = 15*15;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
// bucket minimum and maximum object area
const int BUCKET_MIN_OBJECT_AREA = 60*60;
const int BUCKET_MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
const int BUCKET_AREA_READY_TO_DROP = 37000;

// Calibration Modes
const int SHOW_IMAGES = false;	// true : show directly (Laptop), false: publish it in a ros Node (Odroid)
const int CALIB_THRESHOLD_MODE = false;
const int CALIB_DEPTH_MODE = false;
// Depth conversion 
const float PIXEL_CM = 9;  // X pixels each 1 cm
const int DISPARITY = -4;  // Diference between cameras
// Min and Max depth TODO: Measure this
const int MIN_DEPTH = 60;
const int MAX_DEPTH = 300;
// Manual calibration distance parameters TODO: Figure out the log equation
const int MAX_CM = 30;  // Maximum centimeters = 30
// 0 == Unknown
const int PIXEL_TO_CM [MAX_CM + 1] = 
    {0, 0, 0, 0, 0,
     0, 0, 0, 0, 0,
     MIN_DEPTH, 74, 83, 98, 105,
     112, 120, 129, 136, 142,
     147, 153, 158, 162, 167,
     170, 175, 178, 182, 185,
     MAX_DEPTH};

// Note: Use Auto Color filtering to find HVS values
// Green balls OpenCV, 
// int H_MIN = 31;
// int H_MAX = 70;
// int S_MIN = 123;
// int S_MAX = 255;
// int V_MIN = 80;
// int V_MAX = 255;
// Green balls ROS lab, 
// int H_MIN = 60;
// int H_MAX = 80;
// int S_MIN = 100;
// int S_MAX = 255;
// int V_MIN = 80;
// int V_MAX = 255;
// Green balls ROS (downstaris) final, 
int G_H_MIN = 60;
int G_H_MAX = 74;
int G_S_MIN = 82;
int G_S_MAX = 255;
int G_V_MIN = 80;
int G_V_MAX = 255;
// Orange balls ROS (downstaris) final, 
int O_H_MIN = 107;
int O_H_MAX = 114;
int O_S_MIN = 131;
int O_S_MAX = 255;
int O_V_MIN = 63;
int O_V_MAX = 255;
// Bucket 
int B_H_MIN = 80;
int B_H_MAX = 110;
int B_S_MIN = 63;
int B_S_MAX = 201;
int B_V_MIN = 108;
int B_V_MAX = 255;

// Cameras
Mat right_cam;
Mat left_cam;

//This function gets called whenever a
// trackbar position is changed
void on_trackbar(int, void*)
{
    //for now, this does nothing.
}

// Create Trachbars for Threshold calibration
void createTrackbars() 
{
	//create window for trackbars

	namedWindow(trackbarWindowName, 0);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName, &G_H_MIN, 255, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &G_H_MAX, 255, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &G_S_MIN, 255, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &G_S_MAX, 255, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &G_V_MIN, 255, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &G_V_MAX, 255, on_trackbar);

}

// Erode and dilatate Images
void morphOps(Mat &thresh) 
{

    // Blur Image to readuce noide
    medianBlur(thresh, thresh, 9);  
    // erode and dilatate
    erode(thresh, thresh, Mat(), Point(-1,-1), 1);
    erode(thresh, thresh, Mat(), Point(-1,-1), 1);
    dilate(thresh, thresh, Mat(), Point(-1,-1), 1);
    dilate(thresh, thresh, Mat(), Point(-1,-1), 1);
    erode(thresh, thresh, Mat(), Point(-1,-1), 1);

    /*
    // create structuring element that will be used to "dilate" and "erode" image.
    // the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    // dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));
    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);
    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
    */

}

// Image Segmentation using K-means algorithm
int UberEtiquetado(const Mat &sourceImage, Mat &dest) 
{

    Mat sampImage;
    Mat lab;
    int color=0, nobjects=0;
    int x,y;

    if (sourceImage.empty())
            dest = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());

    // Resize image, this reduces error, do this if we have a lot of error.
    //resize(sourceImage, sampImage,Size(),0.125,0.125,INTER_NEAREST);
    //lab=sampImage.clone();

    lab=sourceImage.clone();


    for (y = 0; y < lab.rows; ++y) { //Recorre los renglones
        for (x = 0; x < lab.cols; ++x) {//Recorre las columnas

            if(lab.at<Vec3b>(y, x)[0] == 255) {
                //Si el de arriba tiene un color assignado, se lo agrega.
                if(lab.at<Vec3b>((y-1), x)[0] < 255 && lab.at<Vec3b>((y-1), x)[0] > 0) {
                    lab.at<Vec3b>(y, x)[0] = lab.at<Vec3b>((y-1), x)[0];
                    lab.at<Vec3b>(y, x)[1] = lab.at<Vec3b>((y-1), x)[1];
                    lab.at<Vec3b>(y, x)[2] = lab.at<Vec3b>((y-1), x)[2];
                //Si el de arriba y uno a la derecha tiene color asignado, se lo agrega
                } else if(lab.at<Vec3b>((y-1), (x+1))[0] < 255 && lab.at<Vec3b>((y-1), (x+1))[0] > 0) {
                    lab.at<Vec3b>(y, x)[0] = lab.at<Vec3b>((y-1), (x+1))[0];
                    lab.at<Vec3b>(y, x)[1] = lab.at<Vec3b>((y-1), (x+1))[1];
                    lab.at<Vec3b>(y, x)[2] = lab.at<Vec3b>((y-1), (x+1))[2];
                //Si el anterior y uno a la izquiera tiene color asignado, se lo agrega
                } else if(lab.at<Vec3b>((y-1), (x-1))[0] < 255 && lab.at<Vec3b>((y-1), (x-1))[0] > 0) {
                    lab.at<Vec3b>(y, x)[0] = lab.at<Vec3b>((y-1), (x-1))[0];
                    lab.at<Vec3b>(y, x)[1] = lab.at<Vec3b>((y-1), (x-1))[1];
                    lab.at<Vec3b>(y, x)[2] = lab.at<Vec3b>((y-1), (x-1))[2];
                //Si el anterior tiene un color assignado, se lo agrega
                } else if(lab.at<Vec3b>(y, (x-1))[0] < 255 && lab.at<Vec3b>(y, (x-1))[0] > 0) {
                    lab.at<Vec3b>(y, x)[0] = lab.at<Vec3b>(y, (x-1))[0];
                    lab.at<Vec3b>(y, x)[1] = lab.at<Vec3b>(y, (x-1))[1];
                    lab.at<Vec3b>(y, x)[2] = lab.at<Vec3b>(y, (x-1))[2];
                //Si el de arriba y dos antes tiene un color assignado, se lo agrega
                } else if(lab.at<Vec3b>((y-1), (x-2))[0] < 255 && lab.at<Vec3b>((y-1), (x-2))[0] > 0) {
                    lab.at<Vec3b>(y, x)[0] = lab.at<Vec3b>((y-1), (x-2))[0];
                    lab.at<Vec3b>(y, x)[1] = lab.at<Vec3b>((y-1), (x-2))[1];
                    lab.at<Vec3b>(y, x)[2] = lab.at<Vec3b>((y-1), (x-2))[2];
                //Si el de arriba y dos adelante tiene un color assignado, se lo agrega
                } else if(lab.at<Vec3b>((y-1), (x+2))[0] < 255 && lab.at<Vec3b>((y+1), (x-2))[0] > 0) {
                    lab.at<Vec3b>(y, x)[0] = lab.at<Vec3b>((y-1), (x+2))[0];
                    lab.at<Vec3b>(y, x)[1] = lab.at<Vec3b>((y-1), (x+2))[1];
                    lab.at<Vec3b>(y, x)[2] = lab.at<Vec3b>((y-1), (x+2))[2];
                //Si no, le asignamos un nuevo color al pixel de 5 en 5, solo en el canal azul
                } else{
                    lab.at<Vec3b>(y, x)[0]= color;
                    lab.at<Vec3b>(y, x)[1]= 255;
                    lab.at<Vec3b>(y, x)[2]= 255;
                    color+=1;
                }
            }
        }
    }

    // Filter depending on Blue color
    inRange(lab, Scalar(0, 255, 255), Scalar(0, 255, 255), dest);
    imshow("Small", dest);

    // Copy reusult image to dest
    nobjects=color;
    printf("Number of objets = %d\n", nobjects);

    return nobjects;

}


// Ball recognition, Segments different objects according to theshold and checks if
// it is a circle.
void detectBalls (vector <Ball> &theBalls, Mat threshold) 
{

    Mat temp;
    threshold.copyTo(temp);

    // Init dest buffer
    //dest = Mat::zeros(threshold.rows, threshold.cols, threshold.type());

    // These two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

    // Use moments method to find our filtered object
    double refArea = 0;
    double T = 0;
    int x = 0;
    int y = 0;
    int objectNum = 0;
    bool objectFound = false;

    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        // If number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                double perimeter = arcLength(contours[index],true);

                // Detect if the object is a circle, The closer T to 1 the closest is to a perfect circle.
                T = 4 * M_PI * (area/pow(perimeter,2));

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && T>=0.58) {
                    
                    x = moment.m10/area;
                    y = moment.m01/area;

                    objectFound = true;
                    objectNum+=1;

                    // Create and Add object ball to the vector
                    Ball ball;
                    ball.setXPos(x);
                    ball.setYPos(y);
                    ball.setArea(area);
                    theBalls.push_back(ball);

                    // Add conntour in the out image (Fill the holes)
                    //drawContours( dest, contours, index, Scalar(255, 255, 255), CV_FILLED, 8, hierarchy );

                } else {
                    objectFound = false; 

                    // Delete the contour in the out image (remove white dots)
                    //drawContours( dest, contours, index, Scalar(0, 0, 0), CV_FILLED, 8, hierarchy );

                }

            }

        } else {
           printf("Too much noise\n"); 
        } 
    }
}

// Bucket Recognition, It returns the center of mass of the bucket and the area in the variable
geometry_msgs::Point detectBucket (Mat threshold, Mat &frame, int &bucket_area) 
{

    Mat temp;
    threshold.copyTo(temp);

    //Position
    geometry_msgs::Point bucket_center;
    bucket_center.x = -1;
    bucket_center.y = -1;

    // Init dest buffer
    //dest = Mat::zeros(threshold.rows, threshold.cols, threshold.type());

    // These two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

    // Use moments method to find our filtered object
    double refArea = 0;
    int x = 0;
    int y = 0;
    int objectNum = 0;
    bool objectFound = false;

    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        // If number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                double perimeter = arcLength(contours[index],true);

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>BUCKET_MIN_OBJECT_AREA) {
                    
                    x = moment.m10/area;
                    y = moment.m01/area;

                    objectFound = true;
                    objectNum+=1;

                    // Add center of biggest bucket area
                    if (area > bucket_area) {
	                    bucket_center.x = x;
	    				bucket_center.y = y;
	    				bucket_area = area;
	    			}

                } else {
                    objectFound = false; 

                }

            }

        } else {
           printf("Too much noise\n"); 
        }
    }

    return bucket_center;
}

void drawBucketPosition(geometry_msgs::Point point_bucket_right, Mat &frame_right, int bucket_area_right,
	geometry_msgs::Point point_bucket_left, Mat &frame_left, int bucket_area_left) 
{

	char textBuff[50];
	int x, y, bucket_area, bucket_area_avg;
	Mat frame;

	if (bucket_area_right!=-1 && bucket_area_left!=-1) {


/*
		// Draw center and add coordinates for bucket right 
		x = point_bucket_right.x;
		y = point_bucket_right.y;
		bucket_area = bucket_area_right;
		frame = frame_right;
		circle(frame,Point(x,y),4,Scalar( 0, 255, 0 ),2, 1);
		sprintf(textBuff, "(%d, %d)", x , y);
		putText(frame,textBuff,Point(x-25,y+15),FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,0,255),1);
		sprintf(textBuff, "%d area", bucket_area);
		putText(frame,textBuff,Point(x-15,y+35),FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,0,0),1);
		//sprintf(textBuff, "%d avg", bucket_area_avg);
		//putText(frame,textBuff,Point(x-15,y+55),FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,255,0),1);

		x = point_bucket_left.x;
		y = point_bucket_left.y;
		bucket_area = bucket_area_left;
		frame = frame_left;
		circle(frame,Point(x,y),4,Scalar( 0, 255, 0 ),2, 1);
		sprintf(textBuff, "(%d, %d)", x , y);
		putText(frame,textBuff,Point(x-25,y+15),FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,0,255),1);
		sprintf(textBuff, "%d area", bucket_area);
		putText(frame,textBuff,Point(x-15,y+35),FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,0,0),1);
		//sprintf(textBuff, "%d avg", bucket_area_avg);
		//putText(frame,textBuff,Point(x-15,y+55),FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,255,0),1);
		*/

		// Average
		x = (point_bucket_right.x + point_bucket_left.x) / 2;
		y = (point_bucket_right.y + point_bucket_left.y) / 2;
		bucket_area_avg = (bucket_area_right + bucket_area_left) / 2;
		frame = frame_left;
		circle(frame,Point(x,y),4,Scalar( 0, 255, 0 ),2, 1);
		sprintf(textBuff, "(%d, %d)", x , y);
		putText(frame,textBuff,Point(x-25,y+15),FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,0,255),1);
		sprintf(textBuff, "%d avg", bucket_area_avg);
		putText(frame,textBuff,Point(x-15,y+35),FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,255,0),1);

	}

}

void drawBallsPosition (vector <Ball> theBalls, Mat &frame) 
{

    int x, y, depth;
    char textBuff[50];

    for (int i =0; i < theBalls.size(); i++) {
        x = theBalls.at(i).getXPos();
        y = theBalls.at(i).getYPos();
        depth = theBalls.at(i).getDepth();

        // Skip drawing if depth == -1
        if (depth == -1) continue;

        // Draw center
        circle(frame,Point(x,y),4,Scalar( 0, 0, 255 ),2, 1);
        // Add coordinates
        sprintf(textBuff, "(%d, %d)", x , y);
        putText(frame,textBuff,Point(x-25,y+15),FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,0,255),1);
        // Add Depth
        if (CALIB_DEPTH_MODE == true) {
            sprintf(textBuff, "%d pixels", depth);
        } else {
            sprintf(textBuff, "%d cm", depth);
        }
        putText(frame,textBuff,Point(x-15,y+35),FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,0,0),1);
    }

    // Print Number of objects
    sprintf(textBuff, "Obj Number = %d", (int) theBalls.size());
    putText(frame,textBuff,Point(0,FRAME_HEIGHT - 10),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);

}

void measureBallsDepth(vector <Ball> &balls_right, vector <Ball> &balls_left) 
{

    // Left camera and right camera shows the same number of balls
    double area_left, area_right;
    int error;
    vector <Ball> *minBalls;
    vector <Ball> *maxBalls;

    // Check if both arrays have balls
    if (balls_right.size() == 0 || balls_left.size() == 0) {
        return;
    } 

    // Check that both frames have the same amount of balls
    if (balls_right.size() > balls_left.size()) {
        // More balls in right camera
        minBalls = &balls_left;
        maxBalls = &balls_right;
    } else  if (balls_left.size() > balls_right.size()){
        // More balls in left camera
        minBalls = &balls_right;
        maxBalls = &balls_left;
    } else {
        // Equal balls in both cameras
        minBalls = &balls_right;
        maxBalls = &balls_left;
    }


    // Match balls of both cameras using minimum area error
    double array [maxBalls->size()];
    for (int j =0; j < minBalls->size(); j++) {

        // Get the errors of ball at j by i
        for (int i =0; i < maxBalls->size(); i++) {

            // If ball not already assigned
            if (maxBalls->at(i).getDepth() == -1 ) {
                error = fabs(minBalls->at(j).getArea() 
                    - maxBalls->at(i).getArea() );
                array[i] = error;
            } else {
                array[i] = INT_MAX;
            }
        }

        // Check who has the smallest error
        double smallest = array[0] ;
        int position = 0;
        for ( int i=1;  i < sizeof(array)/sizeof(array[0]);  ++i ) {
            if ( array[i] < smallest ) {
                smallest = array[i] ;
                position = i;
            }
        }

        // Skip if the MAX error is the smallest
        if (smallest == INT_MAX) continue;

        // We know now the match now calculate the depth
        int depth = abs(minBalls->at(j).getXPos() 
                - maxBalls->at(position).getXPos());

        // Skip if depth is out of range
        if (depth < MIN_DEPTH || depth > MAX_DEPTH ) continue;

        // Invert the range
        depth = MAX_DEPTH - depth;

        if (CALIB_DEPTH_MODE == true ) {
            // Just print the depth to take mesurements
            printf("Depth %d \n", depth);
        } else {
            // Equation HERE: Map the depth to a known measurement x 100 (23.25 = 2325)
            //depth = (int) ((( depth / PIXEL_CM) - DISPARITY) * 100);
            // Manual: Map the depth to cm
            for (int i=0; i < MAX_CM; i++) {
                if (depth >= PIXEL_TO_CM[i] &&  depth < PIXEL_TO_CM[i+1]) {
                    depth = i+1;
                    break;
                }
            }
        }

        // Save the depth for both balls
        minBalls->at(j).setDepth(depth);
        maxBalls->at(position).setDepth(depth);

        // Save the ball number Tag == j
        minBalls->at(j).setTag(j);
        maxBalls->at(position).setTag(j);

        //printf("Error = %f, X1 = %d, X2 = %d, Ball = %i , Depth=  %d \n",smallest, minBalls->at(j).getXPos(), maxBalls->at(position).getXPos(),  j, depth);

    }

}

// Publish Closest ball info to a ROS node.
void publishClosestBallPosition (vector <Ball> balls_right, vector <Ball> balls_left, 
	ros::Publisher pub_track_point_avg, ros::Publisher pub_depth) 
{

	int x, y, depth, tag;
    int x_right, y_right, depth_right, tag_right;
    int x_left, y_left, depth_left, tag_left;
    int smallest_depth = INT_MAX;

    x_right = y_right = depth_right = tag_right = -1;
    x_left = y_left = depth_left = tag_left = -1;

    // Get closest ball and save its data
    for (int i =0; i < balls_right.size(); i++) {
        x = balls_right.at(i).getXPos();
        y = balls_right.at(i).getYPos();
        depth = balls_right.at(i).getDepth();
        tag = balls_right.at(i).getTag();

        if (depth < smallest_depth && depth != -1) {
        	// Swap the smallest
        	smallest_depth = depth;

        	// Save closest ball right
        	x_right = x;
        	y_right = y;
        	depth_right = depth;
        	tag_right = tag;
        }
    }

    // Return if there are no balls
    if (tag_right == -1) return;

    // Get closest left ball data
    for (int i =0; i < balls_left.size(); i ++) {
        x = balls_left.at(i).getXPos();
        y = balls_left.at(i).getYPos();
        depth = balls_left.at(i).getDepth();
        tag = balls_left.at(i).getTag();

    	if (tag == tag_right) {
        	// Save closest ball right
        	x_left = x;
        	y_left = y;
        	depth_left = depth;
        	tag_left = tag;
    	}
    }


    // Print Number of objects
    // printf("Bump ######\n");
    // printf("Positions right: x = %d, y=%d \n", x_right, y_right );
    // printf("Positions left: x = %d, y=%d \n", x_left, y_left);

    // Publish closest ball Average (X,Y) position
	geometry_msgs::Point point_avg;
	point_avg.x = (x_right + x_left) / 2.0;
	point_avg.y = (y_right + y_left) / 2.0;
	pub_track_point_avg.publish(point_avg);

	// Publsh depth
	std_msgs::Int32 msg;
	msg.data = depth_left;
	pub_depth.publish(msg);


}

void publishBucketPosition(geometry_msgs::Point bucket_point_right, geometry_msgs::Point bucket_point_left,
	int area_bucket_right, int area_bucket_left, 
	ros::Publisher pub_bucket_point_avg, ros::Publisher pub_bucket_area) 
{

	if (area_bucket_right != -1 && area_bucket_left != -1) {
		// Publish closest ball Average (X,Y) position
		geometry_msgs::Point point_avg;
		point_avg.x = (bucket_point_right.x + bucket_point_left.x) / 2.0;
		point_avg.y = (bucket_point_right.y + bucket_point_left.y) / 2.0;
		pub_bucket_point_avg.publish(point_avg);

		// Publsh depth
		std_msgs::Int32 msg;
		msg.data = (area_bucket_right + area_bucket_left) / 2;
		pub_bucket_area.publish(msg);
	}

}

/*!
 * \brief callback when the left image is received
 * \param ptr pointer to the image
 */
void leftImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	/* TODO: REPLACE CODE BELOW FOR THIS CODE AND CALIBRATE THRESHOLDS
	try {
		left_cam = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	*/
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = toCvCopy(msg);
	left_cam = cv_ptr->image;
}

/*!
 * \brief callback when the right image is received
 * \param ptr pointer to the image
 */
void rightImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	/* TODO: REPLACE CODE BELOW FOR THIS CODE AND CALIBRATE THRESHOLDS
	try {
		right_cam = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	*/
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = toCvCopy(msg);
	right_cam = cv_ptr->image;
}


// %Tag(INIT)%
int main( int argc, char** argv )
{

    // Ball Objects
    vector <Ball> balls_right;
    vector <Ball> balls_left;

    // Bucket Objects/vars
    geometry_msgs::Point bucket_point_right;
    geometry_msgs::Point bucket_point_left;
    int area_bucket_right = -1;
    int area_bucket_left = -1;

	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;
	ros::Rate r(0.1);
	image_transport::ImageTransport it(n);
	ros::Publisher pub_track_point_avg = n.advertise<geometry_msgs::Point>("track_point_avg", 100);
	ros::Publisher pub_bucket_point_avg = n.advertise<geometry_msgs::Point>("track_bucket_point_avg", 100);
	ros::Publisher pub_depth = n.advertise<std_msgs::Int32>("track_point_depth", 100);
	ros::Publisher pub_bucket_area = n.advertise<std_msgs::Int32>("track_bucket_area", 100);
	image_transport::Subscriber left_sub = it.subscribe("stereo/left/image_rect_color", 1, leftImageCallback);
	image_transport::Subscriber right_sub = it.subscribe("stereo/right/image_rect_color", 1, rightImageCallback);
	image_transport::Publisher opencv_left_pub = it.advertise("track_opencv_result_left", 1);
	image_transport::Publisher opencv_right_pub = it.advertise("track_opencv_result_right", 1);


	Mat frame_right;
    Mat frame_left;
    Mat proc_right;
    Mat proc_left;
    Mat imgMoment;
    Mat threshold_right;
    Mat threshold_left;
    Mat green_threshold_right;
    Mat green_threshold_left;
    Mat orange_threshold_right;
    Mat orange_threshold_left;
    Mat bucket_threshold_right;
    Mat bucket_threshold_left;
	Mat imgWithLines;
    Mat HSV;


	// Maybe move ir to while true
	while (ros::ok()) {

		// If cameras buffers are empty skip the while loop
		if (left_cam.empty() || right_cam.empty() ) {
			ros::spinOnce();
			continue;
		}

		frame_right = right_cam;
		frame_left = left_cam;

    	proc_right = frame_right.clone();
		proc_left  = frame_left.clone();

        ////// IF HSV CALIBRATION MODE //////
        if (CALIB_THRESHOLD_MODE == true) {

            //create slider bars for HSV filtering
            createTrackbars();

            // Convert frame from BGR to HSV colorspace
            cvtColor(proc_right, HSV, COLOR_BGR2HSV);

            // Filter the image leving only the RGB chosen colors
            inRange(HSV, Scalar(G_H_MIN, G_S_MIN, G_V_MIN), Scalar(G_H_MAX, G_S_MAX, G_V_MAX), threshold_right);

            // Convert frame from BGR to HSV colorspace
            cvtColor(proc_left, HSV, COLOR_BGR2HSV);

            // Filter the image leving only the RGB chosen colors
            inRange(HSV, Scalar(G_H_MIN, G_S_MIN, G_V_MIN), Scalar(G_H_MAX, G_S_MAX, G_V_MAX), threshold_left);

            // Perform morphological operations on thresholded image to eliminate noise
        	morphOps(threshold_right);
        	morphOps(threshold_left);

            //show the thresholded image right
            imshow(thresholded_img_right_title.c_str(), threshold_right);

            //show the thresholded image left
            imshow(thresholded_img_left_title.c_str(), threshold_left); 

            // Show Balls detection
	        imshow(left_img_title.c_str(), frame_left);
	        imshow(right_img_title.c_str(), frame_right);
	        

            if(waitKey(30) >= 0) break;

            // Skip the rest of the while loop
            ros::spinOnce();
            continue;
        }


        ////// Right Image Processing  ///////

        // Convert frame from BGR to HSV colorspace
        cvtColor(proc_right, HSV, COLOR_BGR2HSV);

        /// GREEN COLOR 

		// Filter the image leving only the RGB chosen colors
		inRange(HSV, Scalar(G_H_MIN, G_S_MIN, G_V_MIN), Scalar(G_H_MAX, G_S_MAX, G_V_MAX), green_threshold_right);

        // Perform morphological operations on thresholded image to eliminate noise
        morphOps(green_threshold_right);

        // Segment image and detect circles
        detectBalls(balls_right, green_threshold_right);

        /// ORANGE COLOR

		// Filter the image leving only the RGB chosen colors
		inRange(HSV, Scalar(O_H_MIN, O_S_MIN, O_V_MIN), Scalar(O_H_MAX, O_S_MAX, O_V_MAX), orange_threshold_right);

        // Perform morphological operations on thresholded image to eliminate noise
        morphOps(orange_threshold_right);

        // Segment image and detect circles
        detectBalls(balls_right, orange_threshold_right);

        /// BUCKET COLOR

		// Filter the image leving only the RGB chosen colors
		inRange(HSV, Scalar(B_H_MIN, B_S_MIN, B_V_MIN), Scalar(B_H_MAX, B_S_MAX, B_V_MAX), bucket_threshold_right);

        // Perform morphological operations on thresholded image to eliminate noise
        morphOps(bucket_threshold_right);

        // Segment image and detect circles
        bucket_point_right = detectBucket(bucket_threshold_right, frame_right, area_bucket_right);





        ////// Left Image Processing  ///////

        // Convert frame from BGR to HSV colorspace
        cvtColor(proc_left, HSV, COLOR_BGR2HSV);

        /// GREEN COLOR 

        // Filter the image leving only the RGB chosen colors
        inRange(HSV, Scalar(G_H_MIN, G_S_MIN, G_V_MIN), Scalar(G_H_MAX, G_S_MAX, G_V_MAX), green_threshold_left);

        // Perform morphological operations on thresholded image to eliminate noise
        morphOps(green_threshold_left);

        // Segment image and detect circles
        detectBalls(balls_left, green_threshold_left);

        /// ORANGE COLOR

        // Filter the image leving only the RGB chosen colors
        inRange(HSV, Scalar(O_H_MIN, O_S_MIN, O_V_MIN), Scalar(O_H_MAX, O_S_MAX, O_V_MAX), orange_threshold_left);

        // Perform morphological operations on thresholded image to eliminate noise
        morphOps(orange_threshold_left);

        // Segment image and detect circles
        detectBalls(balls_left, orange_threshold_left);

        /// BUCKET COLOR

        // Filter the image leving only the RGB chosen colors
        inRange(HSV, Scalar(B_H_MIN, B_S_MIN, B_V_MIN), Scalar(B_H_MAX, B_S_MAX, B_V_MAX), bucket_threshold_left);

        // Perform morphological operations on thresholded image to eliminate noise
        morphOps(bucket_threshold_left);

        // Segment image and detect circles
        bucket_point_left = detectBucket(bucket_threshold_left, frame_left, area_bucket_left);





        ////// Stereo Image Processing  ///////

        // Measure depth of each ball 
        measureBallsDepth(balls_right, balls_left);

        // Draw balls position and display data
        drawBallsPosition(balls_right, frame_right);

        // Draw balls position and display data
        drawBallsPosition(balls_left, frame_left);

        // Draw Bucket Position
        drawBucketPosition(bucket_point_right, frame_right, area_bucket_right,
         bucket_point_left, frame_left, area_bucket_left);

        

        // Publish Point data of closest ball
        publishClosestBallPosition(balls_right, balls_left, 
        	pub_track_point_avg, pub_depth);

        // Publish Point data of the bucket
        publishBucketPosition(bucket_point_right, bucket_point_left, 
        	area_bucket_right, area_bucket_left,
        	pub_bucket_point_avg, pub_bucket_area);


        ////// Show STUFF ///////

        // Show or publish images
        if (SHOW_IMAGES == true) {

	        // Show Thresholded left and right images
	        imshow(thresholded_img_left_title.c_str(), bucket_threshold_left); 
	        imshow(thresholded_img_right_title.c_str(), bucket_threshold_right); 

	        // Show Balls detection
	        imshow(left_img_title.c_str(), frame_left);
	        imshow(right_img_title.c_str(), frame_right);
	        
	    } else {

	    	// Publish Balls detection
	    	sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_left).toImageMsg();
	    	sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_right).toImageMsg();
	    	opencv_left_pub.publish(msg_left);
	    	opencv_right_pub.publish(msg_right);

	    }

        // Clear vector balls
        balls_right.clear();
        balls_left.clear();
        // Clear bucket area
        area_bucket_right = -1;
    	area_bucket_left = -1;

		ros::spinOnce();

		if(waitKey(30) >= 0) break;

	}

	destroyAllWindows();

	return 0;
}
