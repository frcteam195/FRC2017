//
//  main.cpp
//  OpenCVTestSingleImage
//
//  Created by Robert Hilton on 2/12/17.
//  Copyright © 2017 Hilton Tuning. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>

#define DEBUG
#define SAVE_JPG
//#define JETSON_RUN

using namespace std;
using namespace cv;

const double dilation_size = 1.75;
const Scalar color = Scalar(255, 255, 255);
double focalLength = (31.48*111)/4;
double pictureWidth = 640;
const double cameraFOV = 170/3;
Mat drawingOut;

Mat element = getStructuringElement(MORPH_RECT,
									Size(2 * dilation_size + 1, 2 * dilation_size + 1),
									Point(dilation_size, dilation_size));

int main(int argc, const char * argv[]) {

	Mat out, canny_output, dst, procImg;
#ifdef JETSON_RUN
	Mat frame = imread("/home/ubuntu/KnightVision/imageStore/ImgOri255.jpg");
#else
	Mat frame = imread("/Users/roberthilton/Desktop/ls13/ImgOri64.jpg");
#endif
	
	//blur(frame,frame,cv::Size(3,3));
	
#ifndef JETSON_RUN
	imshow("Original", frame);
#endif
	
	threshold(frame,procImg,180,255,CV_THRESH_BINARY);
	
	
	imshow("Threshold", procImg);
	dilate(procImg, dst, element); // Dilate image to fill in any imperfections
	cvtColor(dst, dst, COLOR_BGR2HSV);
	imshow("Dialated", dst);

	inRange(dst, Scalar(40, 0, 0), Scalar(65, 255, 255), dst); // BGR threshold to ID target
	
	//inRange(frame, Scalar(120, 95, 70), Scalar(135, 105, 100), procImg);
	imshow("Processed", dst);
	
	
	
	//imshow("Processed Image", procImg);
	
		//imshow("Dilated Image", dst);
	
	
	// Set up the detector with default parameters.
	// Detect blobs.
	/*
	std::vector<KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();
	detector->detect( dst, keypoints );
 
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	drawKeypoints( dst, keypoints, canny_output, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 */
	//imshow("keypoints", canny_output );
	//waitKey(0);
	Canny(dst, canny_output, 0, 0); // Detect edges of target - Add 6ms
	//waitKey(30);
	//imshow("Canny", canny_output);
	
	vector<vector<Point> > contoursVector;
	
	// Generate contours from the edges - Add 1ms
	findContours(canny_output, contoursVector, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE , Point(0, 0));
	
	vector<RotatedRect> boxes(contoursVector.size());
	
	
	//Draw image with found contours for viewing/testing, fill vector with the smallest
	//rotated rectangles that contain the contours
	
#ifdef SAVE_JPG
	drawingOut = Mat::zeros(canny_output.size(), CV_32F);
#endif
	for(int i = 0; i < contoursVector.size(); i++) {
#ifdef SAVE_JPG
		drawContours(drawingOut, contoursVector, i, color, 1, 4, noArray(), 0, Point());
#endif
		boxes[i] = minAreaRect(contoursVector[i]);
	}
	
	for (vector<RotatedRect>::iterator it = boxes.begin(); it!=boxes.end(); )
	{
		cout << it->size.area() << endl;
		if (it->size.area() < 200)
			it=boxes.erase(it);
		else
			++it;
	}
	
	int particle1 = -1;
	int particle2 = -1;
	int lowestDiff = 10000;
	int currentDiff = 10000;
	int stripeHeightImg = 0;
	for(int i = 0; i < boxes.size(); i++) {
		for(int k = 0; k < boxes.size(); k++) {
			int heightI = 0;
			int widthI = 0;
			int heightK = 0;
			int widthK = 0;
			
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
#ifdef DEBUG
			//cout << "i area:" << boxes[i].size.area() << endl;
			//cout << "k area:" << boxes[k].size.area() << endl;
#endif
			if(i != k) {
				if (boxes[i].size.area() < 50)
				{
					k = (int)boxes.size() + 1;
#ifdef DEBUG
					//cout << "Skipping particle i" << i << endl;
#endif
					continue;
				}
				if (boxes[k].size.area() < 50)
				{
#ifdef DEBUG
					//cout << "Skipping particle k" << k << endl;
#endif
					continue;
				}
				currentDiff = abs(boxes[i].center.x - boxes[k].center.x);
				if(currentDiff < lowestDiff) {
					lowestDiff = currentDiff;
					if(abs(widthI - widthK) < 20 && lowestDiff < 10 && widthI > heightI && widthK > heightK && abs(boxes[i].center.y - boxes[k].center.y) < 50) {
						particle1 = i;
						particle2 = k;
						
						if (heightI > heightK)
							stripeHeightImg = heightI;
						else
							stripeHeightImg = heightK;
						
#ifdef DEBUG
						cout << "Particle 1 Angle: " << boxes[i].angle << endl;
						cout << "Particle 2 Angle: " << boxes[k].angle << endl;
						cout << "Particle 1 Area: " << boxes[i].size.area() << endl;
						cout << "Particle 2 Area: " << boxes[k].size.area() << endl;
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
		;//return 1;
	
	
	
	
	double averageCenterX = 0;
	double deviation = 0;
	bool targetFound;
	double targetDistance = (4 * focalLength)/stripeHeightImg;
	
#ifdef DEBUG
	
	cout << "Focal Length: " << focalLength << endl;
	cout << "stripe height: " << stripeHeightImg << endl;
	for(int i = 0; i < filteredBoxes.size(); i++) {
		Point2f rect_points[4];
		filteredBoxes[i].points(rect_points);
		for(int j = 0; j < 4; j++)
			line(drawingOut, rect_points[j], rect_points[(j+1)%4], color, 1, CV_AA);
	}
#ifndef JETSON_RUN
	
	imshow("Not Original", drawingOut);
#else
	imwrite("/home/ubuntu/OpenCVTestSingleImage/outputImgTest.jpg", drawingOut);
#endif
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
	
#ifndef JETSON_RUN
	bool runKey = true;
	while (runKey)
	{
		int keyCode = waitKey(30);
		if (keyCode != 255)
			runKey = false;
	}
#endif
    return 0;
}
