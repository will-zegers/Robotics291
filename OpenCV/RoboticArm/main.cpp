#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <limits.h>
#include "ball.h"

// Name Spaces
using namespace cv;
using namespace std;

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
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

// Calibration Modes
const int CALIB_THRESHOLD_MODE = false;
const int CALIB_DEPTH_MODE = false;
// Depth conversion 
const float PIXEL_CM = 9;  // X pixels each 1 cm
const int DISPARITY = -4;  // Diference between cameras
// Min and Max depth TODO: Measure this
const int MIN_DEPTH = 58;
const int MAX_DEPTH = 300;
// Manual calibration distance parameters TODO: Figure out the log equation
const int MAX_CM = 30;  // Maximum centimeters = 30
// 0 == Unknown
const int PIXEL_TO_CM [MAX_CM + 1] = 
    {0, 0, 0, 0, 0,
     0, 0, 0, 0, 0,
     0, MIN_DEPTH, 67, 79, 94,
     105, 116, 125, 134, 142,
     149, 154, 160, 165, 170,
     175, 179, 182, 186, 190,
     MAX_DEPTH};

// Note: Use Auto Color filtering to find HVS values
// Green balls, 
int H_MIN = 31;
int H_MAX = 70;
int S_MIN = 123;
int S_MAX = 255;
int V_MIN = 80;
int V_MAX = 255;

//This function gets called whenever a
// trackbar position is changed
void on_trackbar(int, void*)
{
    //for now, this does nothing.
}

// Create Trachbars for Threshold calibration
void createTrackbars() {
	//create window for trackbars

	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, 255, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, 255, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, 255, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, 255, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, 255, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, 255, on_trackbar);

}

// Erode and dilatate Images
void morphOps(Mat &thresh) {

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
int UberEtiquetado(const Mat &sourceImage, Mat &dest) {

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
void detectBalls (vector <Ball> &theBalls, Mat threshold, Mat &dest){

    Mat temp;
    threshold.copyTo(temp);

    // Init dest buffer
    dest = Mat::zeros(threshold.rows, threshold.cols, threshold.type());

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
                if(area>MIN_OBJECT_AREA && T>=0.78) {
                    
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
                    drawContours( dest, contours, index, Scalar(255, 255, 255), CV_FILLED, 8, hierarchy );

                } else {
                    objectFound = false; 

                    // Delete the contour in the out image (remove white dots)
                    drawContours( dest, contours, index, Scalar(0, 0, 0), CV_FILLED, 8, hierarchy );

                }

            }

        } else {
           printf("Too much noise\n"); 
        } 
    }
}

void drawBallsPosition (vector <Ball> theBalls, Mat &frame) {

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

void measureBallsDepth(vector <Ball> &balls_right, vector <Ball> &balls_left) {

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
            printf("Depth %d \n", depth);
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

        // Save the depth
        minBalls->at(j).setDepth(depth);
        maxBalls->at(position).setDepth(depth);

        //printf("Error = %f, X1 = %d, X2 = %d, Ball = %i , Depth=  %d \n",smallest, minBalls->at(j).getXPos(), maxBalls->at(position).getXPos(),  j, depth);

    }

}


int main(int argc, char* argv[])
{

    // Ball Objects
    vector <Ball> balls_right;
    vector <Ball> balls_left;

    VideoCapture right_cam = VideoCapture(1);
    VideoCapture left_cam = VideoCapture(2);

    // Check if cameras are connected 
    if(!right_cam.isOpened() || !left_cam.isOpened()) {  
        printf("One camera is not connected :( \n");
        return -1;
    }

    // Set cameras to low resolution
    right_cam.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    right_cam.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    left_cam.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    left_cam.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    Mat frame_right;
    Mat frame_left;
    Mat proc_right;
    Mat proc_left;
    Mat imgMoment;
    Mat threshold_right;
    Mat threshold_left;
	Mat imgWithLines;
    Mat HSV;
    Mat segmented_right;
    Mat segmented_left;

    while (true)
    {

        //printf("Bump ###### \n");
    	
    	right_cam.read(frame_right);
    	left_cam.read(frame_left);	

    	//imshow(left_img_title.c_str(), frame_right);
        //imshow(right_img_title.c_str(), frame_right);

		proc_right = frame_right.clone();
		proc_left  = frame_left.clone();

        ////// IF HSV CALIBRATION MODE //////
        if (CALIB_THRESHOLD_MODE == true) {

            //create slider bars for HSV filtering
            createTrackbars();

            // Convert frame from BGR to HSV colorspace
            cvtColor(proc_right, HSV, COLOR_BGR2HSV);

            // Filter the image leving only the RGB chosen colors
            inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold_right);

            // Convert frame from BGR to HSV colorspace
            cvtColor(proc_left, HSV, COLOR_BGR2HSV);

            // Filter the image leving only the RGB chosen colors
            inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold_left);

            //show the thresholded image right
            imshow(thresholded_img_right_title.c_str(), threshold_right);

            //show the thresholded image left
            imshow(thresholded_img_left_title.c_str(), threshold_left); 

            if(waitKey(30) >= 0) break;

            // Skip the rest of the while loop
            continue;
        }


        ////// Right Image Processing  ///////

        // Blur the image a little before filtering 
        medianBlur(proc_right, proc_right, 9);  

        // Convert frame from BGR to HSV colorspace
        cvtColor(proc_right, HSV, COLOR_BGR2HSV);

		// Filter the image leving only the RGB chosen colors
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold_right);

        // Perform morphological operations on thresholded image to eliminate noise
        morphOps(threshold_right);

        // Segment image and detect circles
        detectBalls(balls_left, threshold_right, segmented_right);


        ////// Left Image Processing  ///////

        // Blur the image a little before filtering 
        medianBlur(proc_left, proc_left, 9);  

        // Convert frame from BGR to HSV colorspace
        cvtColor(proc_left, HSV, COLOR_BGR2HSV);

        // Filter the image leving only the RGB chosen colors
        inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold_left);

        // Perform morphological operations on thresholded image to eliminate noise
        morphOps(threshold_left);

        // Segment image and detect circles
        detectBalls(balls_right, threshold_left, segmented_left);


        ////// Stereo Image Processing  ///////

        // Measure depth of each ball 
        measureBallsDepth(balls_right, balls_left);

        // Draw balls position and display data
        drawBallsPosition(balls_right, frame_left);

        // Draw balls position and display data
        drawBallsPosition(balls_left, frame_right);


        ////// Show STUFF ///////

        // Show Thresholded left and right images
        imshow(thresholded_img_left_title.c_str(), threshold_left); 
        imshow(thresholded_img_right_title.c_str(), threshold_right); 

        // Show Balls detection
        imshow(right_img_title.c_str(), frame_right);
        imshow(left_img_title.c_str(), frame_left);

        // Clear vector balls
        balls_right.clear();
        balls_left.clear();

		if(waitKey(30) >= 0) break;
    }

    destroyAllWindows();

    return(0);
}