#include "opencv2/opencv.hpp"
#include "KnightCamera.h"
#include "UDPSocket.h"
#include "SimpleGPIO.h"
#include <signal.h>
#include <iostream>
#include <thread>

//#define SAVE_JPG
//#define DEBUG

using namespace cv;
using namespace std;

Mat frame, prevFrame, img, procImg, dst, canny_output, drawing;

string saveStr;
string saveStrOri;

const double dilation_size = 1.6;
const Scalar color = Scalar(255, 255, 255);

Mat element = getStructuringElement(MORPH_RECT,
									Size(2 * dilation_size + 1, 2 * dilation_size + 1),
									Point(dilation_size, dilation_size));

int particle1 = -1;
int particle2 = -1;
int lowestDiff, currentDiff;
double averageCenterX;

const unsigned int LEDGPIO = 57;
const unsigned int CAMGPIO = 81;

double pictureWidth;

//double focalLength = (186.6520701734*24/8.5); //24 inch distance
double focalLength = (31.48*111)/4;
double stripeHeightImg = 0;

const int minBlobArea = 300;
const int minContourSize = 20;
const double cameraFOV = 170/3;
const double allowedError = .45;
bool onTarget = false;
bool targetFound = false;
double deviation = 0;
double targetDistance = 0;
unsigned long sequence = 0;
bool runLoop = true;

double heightI, widthI, angleI, heightK, widthK, angleK;

UDPSocket *turretUDP;
KnightCamera *turretCam;

VideoCapture *turretCamCap;

int init() {
	turretUDP = new UDPSocket(5801);
	turretUDP->init();
	
	gpio_export(LEDGPIO);
	gpio_set_dir(LEDGPIO, PIN_DIRECTION::OUTPUT_PIN);
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	
	system("v4l2-ctl -d 0 --set-fmt-video=width=640,height=480,pixelformat=0 --set-parm=120");
	system("v4l2-ctl -d 0  --set-ctrl=exposure_auto_priority=1,exposure_auto=1,exposure_absolute=3,brightness=-64,white_balance_temperature_auto=0,white_balance_temperature=6500");
	
	turretCamCap = new VideoCapture(0); // open the default camera
	if(!turretCamCap->isOpened())  // check if we succeeded
		return -1;
	
	turretCamCap->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	turretCamCap->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	turretCamCap->set(CV_CAP_PROP_FPS, 120);
	 
	 
	turretCam = new KnightCamera(turretCamCap);
	
	runLoop = true;
	
	this_thread::sleep_for(chrono::milliseconds(1000));
	
	turretCam->start();
	this_thread::sleep_for(chrono::milliseconds(500));
	frame = turretCam->getLatestFrame();
	pictureWidth = 640;
	
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	this_thread::sleep_for(chrono::milliseconds(200));
	gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
	//this_thread::sleep_for(chrono::milliseconds(200));
	//gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
	
	//turretUDP->start();
	
	cout << "Full init finished!" << endl;
	
	return 0;
}

void interruptHandler(int s){
	//printf("Caught signal %d\n",s);
	if (s == 2) {
		gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
		gpio_unexport(LEDGPIO);
		turretCam->stop();
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
	
	/*
	while(init() != 0) {
		cout << "Init fail!" << endl;
		try {
			if(turretCamCap->isOpened())
				turretCamCap->release();
			turretCam = nullptr;
		} catch (Exception ex) {
			
		}
	}*/
	
	init();
#ifdef SAVE_JPG
	int zIndex = 0;
#endif
	while(runLoop) {
		try {
			frame = turretCam->getLatestFrame();
			
			if(sum(frame != prevFrame) == Scalar(0,0,0,0)) {
				continue;
			}
			
			prevFrame = frame;
					/*
			//imshow("Unprocessed Image", frame); // Show unprocessed image
			//waitKey(30);
			//cvtColor(frame, frame, COLOR_RGB2HSV);
			//inRange(frame, Scalar(60, 135, 0), Scalar(200, 255, 200), procImg); // BGR threshold to ID target
			inRange(frame, Scalar(0, 100, 0), Scalar(90, 245, 20), procImg); // BGR threshold to ID target
			
			
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
			//rotated rectangles that contain the contours*/
			
			
			threshold(frame,procImg,180,255,CV_THRESH_BINARY);
			dilate(procImg, dst, element); // Dilate image to fill in any imperfections
			cvtColor(dst, dst, COLOR_BGR2HSV);
			inRange(dst, Scalar(40, 0, 0), Scalar(65, 255, 255), dst); // HSV threshold to ID target
			Canny(dst, canny_output, 0, 0); // Detect edges of target - Add 6ms
			vector<vector<Point> > contoursVector;
			findContours(canny_output, contoursVector, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE , Point(0, 0));
			
			for (vector<vector<Point> >::iterator it = contoursVector.begin(); it!=contoursVector.end(); )
			{
				if (it->size() < minContourSize)
					it=contoursVector.erase(it);
				else
					++it;
			}
			
			vector<RotatedRect> boxes(contoursVector.size());
			
#ifdef SAVE_JPG
			Mat drawingOut = Mat::zeros(canny_output.size(), CV_32F);
			saveStrOri = "imageStore/ImgOri" + to_string(++zIndex) + ".jpg";
			imwrite( saveStrOri, frame );
#endif
			for(int i = 0; i < contoursVector.size(); i++) {
#ifdef SAVE_JPG
				drawContours(drawingOut, contoursVector, i, color, 1, 4, noArray(), 0, Point());
				saveStrOri = "imageStore/ImgContours" + to_string(zIndex) + ".jpg";
				imwrite( saveStrOri, drawingOut );
#endif
				boxes[i] = minAreaRect(contoursVector[i]);
			}
			
			for (vector<RotatedRect>::iterator it = boxes.begin(); it!=boxes.end(); )
			{
				if (it->size.area() < minBlobArea)
					it=boxes.erase(it);
				else
					++it;
			}
			
			particle1 = -1;
			particle2 = -1;
			lowestDiff = 10000;
			currentDiff = 10000;
			stripeHeightImg = 0;
			for(int i = 0; i < boxes.size(); i++) {
				for(int k = 0; k < boxes.size(); k++) {
					heightI = 0;
					widthI = 0;
					heightK = 0;
					widthK = 0;
					if (boxes[i].size.height > boxes[i].size.width) {
						heightI = boxes[i].size.width;
						widthI = boxes[i].size.height;
					} else {
						heightI = boxes[i].size.height;
						widthI = boxes[i].size.width;
					}
					if (boxes[k].size.height > boxes[k].size.width) {
						heightK = boxes[k].size.width;
						widthK = boxes[k].size.height;
					} else {
						heightK = boxes[k].size.height;
						widthK = boxes[k].size.width;
					}
					
					angleI = boxes[i].angle;
					angleK = boxes[k].angle;
					
					if(i != k) {
						if (boxes[i].size.area() < 40)
						{
							k = (int)boxes.size() + 1;
#ifdef DEBUG
							cout << "Skipping particle i" << i << endl;
#endif
							continue;
						}
						if (boxes[k].size.area() < 40)
						{
#ifdef DEBUG
							cout << "Skipping particle k" << k << endl;
#endif
							continue;
						}
						
						
#ifdef DEBUG
						cout << "JUST INFO" << endl;
						cout << "Particle 1 Angle: " << boxes[i].angle << endl;
						cout << "Particle 2 Angle: " << boxes[k].angle << endl;
						cout << "Particle 1: " << boxes[i].size.area() << endl;
						cout << "Particle 2: " << boxes[k].size.area() << endl;
						cout << "Particle 1 height: " << heightI << endl;
						cout << "Particle 1 width: " << widthI << endl;
						cout << "Particle 2 height: " << heightK << endl;
						cout << "Particle 2 width: " << widthK << endl;
						cout << "Width Difference: " << abs(widthI - widthK) << endl;
						cout << "Center of X difference: " << lowestDiff << endl;
						cout << "Center of Y difference: " << boxes[i].center.y - boxes[k].center.y << endl;
						cout << "END JUST INFO" << endl;
#endif
						
						currentDiff = abs(boxes[i].center.x - boxes[k].center.x);
						if(currentDiff < lowestDiff) {
							lowestDiff = currentDiff;
							if(abs(widthI - widthK) < 20 && lowestDiff < 10 && widthI > heightI && widthK > heightK && abs(boxes[i].center.y - boxes[k].center.y) < 50) {
								//&& abs(angleI - angleK) < 6
								particle1 = i;
								particle2 = k;
								
								if (heightI > heightK)
									stripeHeightImg = heightI;
								else
									stripeHeightImg = heightK;
								
#ifdef DEBUG
								cout << "Particle 1 Angle: " << boxes[i].angle << endl;
								cout << "Particle 2 Angle: " << boxes[k].angle << endl;
								cout << "Particle 1: " << boxes[i].size.area() << endl;
								cout << "Particle 2: " << boxes[k].size.area() << endl;
								cout << "Particle 1 height: " << heightI << endl;
								cout << "Particle 1 width: " << widthI << endl;
								cout << "Particle 2 height: " << heightK << endl;
								cout << "Particle 2 width: " << widthK << endl;
								cout << "Width Difference: " << abs(widthI - widthK) << endl;
								cout << "Center of X difference: " << lowestDiff << endl;
								cout << "Center of Y difference: " << boxes[i].center.y - boxes[k].center.y << endl;
#endif
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
			else
				continue;
			

			
			
			averageCenterX = 0;
			
			targetDistance = (4 * focalLength)/stripeHeightImg;
			
#ifdef DEBUG
			
			cout << "Focal Length: " << focalLength << endl;
			cout << "stripe height: " << stripeHeightImg << endl;
#endif
			
			for(int i = 0; i < filteredBoxes.size(); i++) {
				averageCenterX += filteredBoxes[i].center.x;
			}
			
			if (filteredBoxes.size() > 0)
				averageCenterX /= filteredBoxes.size();
			else
				averageCenterX = 0;
			
			if (particle1 < 0 || particle2 < 0) {
				deviation = 0;
				targetFound = false;
			}
			else {
				deviation = (averageCenterX - (pictureWidth / 2)) / pictureWidth * cameraFOV;
				targetFound = true;
			}
			
#ifdef SAVE_JPG
			for(int i = 0; i < filteredBoxes.size(); i++) {
				Point2f rect_points[4];
				filteredBoxes[i].points(rect_points);
				for(int j = 0; j < 4; j++)
					line(drawingOut, rect_points[j], rect_points[(j+1)%4], color, 1, CV_AA);
			}
			
			
			saveStr = "imageStore/Img" + to_string(zIndex) + ".jpg";
			//if (abs(deviation) > 10)
			//{
				imwrite( saveStr, drawingOut );
			//}
#endif
			
			
			onTarget = abs(deviation) <= allowedError;
#ifdef DEBUG
			cout << targetDistance << endl;
#endif
			turretUDP->udpSendSynchronous(deviation, targetDistance, onTarget, targetFound, sequence++);
			
		} catch(Exception ex) {
			/*
			cout << "Error occurred! Trying reinit." << endl;
			while(init() != 0) {
				cout << "Init fail!" << endl;
				try {
					if(turretCamCap->isOpened())
						turretCamCap->release();
					turretCam = nullptr;
				} catch (Exception ex) {
					
				}
			}*/
		}
	}
	
	gpio_unexport(LEDGPIO);
	turretCam->stop();
	return 0;
}

