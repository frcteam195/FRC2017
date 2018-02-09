#include "opencv2/opencv.hpp"
#include "KnightCamera.h"
#include "UDPSocket.h"
#include "SimpleGPIO.h"
#include <signal.h>
#include <iostream>
#include <thread>

#define SAVE_JPG

using namespace cv;
using namespace std;

Mat frame, img, procImg, dst, canny_output, drawing;

const double dilation_size = 1;
const Scalar color = Scalar(255, 255, 255);

Mat element = getStructuringElement(MORPH_RECT,
	Size(2 * dilation_size + 1, 2 * dilation_size + 1),
	Point(dilation_size, dilation_size));

int particle1 = -1;
int particle2 = -1;
int lowestDiff, currentDiff;
double averageCenterX;

unsigned int LEDGPIO = 57;
unsigned int CAMGPIO = 81;

int pictureWidth;

const double cameraFOV = 170/3;
const double allowedError = 2;
bool onTarget = false;
double deviation = 0;
bool runLoop = true;

UDPSocket *turretUDP;
KnightCamera *turretCam;
KnightCamera *gearCam;

VideoCapture *turretCamCap;
VideoCapture *gearCamCap;

int init() {
	turretUDP = new UDPSocket(5801);
	turretUDP->init();

	gpio_export(LEDGPIO);
	gpio_set_dir(LEDGPIO, PIN_DIRECTION::OUTPUT_PIN);
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);

	gpio_export(CAMGPIO);
	gpio_set_dir(CAMGPIO, PIN_DIRECTION::OUTPUT_PIN);
	gpio_set_value(CAMGPIO, PIN_VALUE::LOW);

	system("v4l2-ctl -d 0 --set-fmt-video=width=640,height=480,pixelformat=0 --set-parm=120");
	system("v4l2-ctl -d 0  --set-ctrl=exposure_auto_priority=1,exposure_auto=1,exposure_absolute=20,brightness=-64");

    turretCamCap = new VideoCapture(0); // open the default camera
    if(!turretCamCap->isOpened())  // check if we succeeded
        return -1;

    gpio_set_value(CAMGPIO, PIN_VALUE::HIGH);
    this_thread::sleep_for(chrono::milliseconds(1000));

	system("v4l2-ctl -d 1 --set-fmt-video=width=640,height=480,pixelformat=0 --set-parm=120");
	system("v4l2-ctl -d 1  --set-ctrl=exposure_auto_priority=1,exposure_auto=1,exposure_absolute=20,brightness=-64");

    gearCamCap = new VideoCapture(1); // open the default camera
    if(!gearCamCap->isOpened())  // check if we succeeded
        ;//return -1;

	turretCamCap->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	turretCamCap->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	turretCamCap->set(CV_CAP_PROP_FPS, 120);

	gearCamCap->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	gearCamCap->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	gearCamCap->set(CV_CAP_PROP_FPS, 120);

	turretCam = new KnightCamera(turretCamCap);
	gearCam = new KnightCamera(gearCamCap);

	runLoop = true;

	this_thread::sleep_for(chrono::milliseconds(1000));

	turretCam->start();
	this_thread::sleep_for(chrono::milliseconds(500));
	frame = turretCam->getLatestFrame();
	pictureWidth = frame.cols;
	cout << pictureWidth << endl;
	return 0;
}

void interruptHandler(int s){
   //printf("Caught signal %d\n",s);
   if (s == 2) {
   		gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
   		gpio_set_value(CAMGPIO, PIN_VALUE::LOW);
		gpio_unexport(LEDGPIO);
		gpio_unexport(CAMGPIO);
		turretCam->stop();
		gearCam->stop();
		exit(1); 
   }
}


int main(int argc, char** argv) {
	//Handle Ctrl-C Signals
	struct sigaction sigIntHandler;
 	sigIntHandler.sa_handler = interruptHandler;
 	sigemptyset(&sigIntHandler.sa_mask);
 	sigIntHandler.sa_flags = 0;
  	sigaction(SIGINT, &sigIntHandler, NULL);
  	////////////////////////


	while(init() != 0) {
		cout << "Init fail!" << endl;
		try {
			if(turretCamCap->isOpened())
        		turretCamCap->release();
        	if(gearCamCap->isOpened())
        		turretCamCap->release();
        	turretCam = nullptr;
        	gearCam = nullptr;
		} catch (Exception ex) {

		}
	}
	int zIndex = 0;
	while(runLoop) {
		try {
			frame = turretCam->getLatestFrame();
			//imshow("Unprocessed Image", frame); // Show unprocessed image
		    //waitKey(30);
		    
			inRange(frame, Scalar(60, 135, 0), Scalar(200, 255, 200), procImg); // BGR threshold to ID target

			//imshow("Processed Image", procImg);
			dilate(procImg, dst, element); // Dilate image to fill in any inperfections

			//imshow("Dilated Image", dst);

			Canny(dst, canny_output, 0, 0); // Detect edges of target - Add 6ms
 			//waitKey(30);
			//imshow("Canny", canny_output);

			vector<vector<Point> > contoursVector;

			// Generate contours from the edges - Add 1ms
			findContours(canny_output, contoursVector, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE , Point(0, 0));

		    vector<RotatedRect> boxes(contoursVector.size());

			
			//Draw image with found contours for viewing/testing, fill vector with the smallest
			//rotated rectangles that contain the contours
			
#ifdef SAVE_JPG
			Mat drawingOut = Mat::zeros(canny_output.size(), CV_32F);
#endif
		    for(int i = 0; i < contoursVector.size(); i++) {
#ifdef SAVE_JPG
		        drawContours(drawingOut, contoursVector, i, color, 1, 4, noArray(), 0, Point());
#endif
		        boxes[i] = minAreaRect(contoursVector[i]);
		    }

			particle1 = -1;
			particle2 = -1;
			lowestDiff = 10000;
			currentDiff = 10000;

			for(int i = 0; i < boxes.size(); i++)
				if(boxes[i].size.area() < 250)
					boxes.erase(boxes.begin() + i);
		
		    for(int i = 0; i < boxes.size(); i++) {
		    	for(int k = 0; k < boxes.size(); k++) {
		    		if(i != k) {
		    			currentDiff = abs(boxes[i].center.x - boxes[k].center.x);
		    			if(currentDiff < lowestDiff) {
		    				lowestDiff = currentDiff;
		    				if(abs(boxes[i].size.width - boxes[k].size.width) < 20) {
			    				particle1 = i;
			    				particle2 = k;
		    				}
		    			}
		    		}
		    	}
		    }

		    vector<RotatedRect> filteredBoxes;

		   	if(particle1 != -1 && particle2 != -1) {
		   		filteredBoxes.push_back(boxes[particle1]);
		   		filteredBoxes.push_back(boxes[particle2]);
		   	}

#ifdef SAVE_JPG
		   	for(int i = 0; i < filteredBoxes.size(); i++) {
				Point2f rect_points[4];
				filteredBoxes[i].points(rect_points);
				for(int j = 0; j < 4; j++)
					line(drawingOut, rect_points[j], rect_points[(j+1)%4], color, 1, CV_AA);
			}


			string saveStr = "imageStore/Img" + to_string(zIndex++) + ".jpg";
 			imwrite( saveStr, drawingOut );
 #endif


			averageCenterX = 0;

			for(int i = 0; i < filteredBoxes.size(); i++) {
				//cout << filteredBoxes[i].center.x << endl;
				averageCenterX += filteredBoxes[i].center.x;
			}

			if (filteredBoxes.size() > 0)
				averageCenterX /= filteredBoxes.size();
			else
				averageCenterX = 0;

			deviation = (averageCenterX - (pictureWidth / 2)) / pictureWidth * cameraFOV;
			onTarget = abs(deviation) <= allowedError;
			//cout << deviation << endl;
			turretUDP->udpSendSynchronous(deviation, onTarget);
			
			
		} catch(Exception ex) {
			cout << "Error occurred! Trying reinit." << endl;
			while(init() != 0) {
				cout << "Init fail!" << endl;
				try {
					if(turretCamCap->isOpened())
		        		turretCamCap->release();
		        	if(gearCamCap->isOpened())
		        		turretCamCap->release();
		        	turretCam = nullptr;
		        	gearCam = nullptr;
				} catch (Exception ex) {

				}
			}
		}
	}

	gpio_unexport(LEDGPIO);
	gpio_unexport(CAMGPIO);
	turretCam->stop();
	gearCam->stop();
    return 0;
}

