#include "KnightCamera.h"
#include <iostream>

KnightCamera::KnightCamera(VideoCapture *cap) {
	this->cap = cap;
	runThread = false;
}

void KnightCamera::start() {
	runThread = true;
	grabFrameThread = thread(&KnightCamera::grabFrame, this);
}

void KnightCamera::stop() {
	runThread = false;
	if(grabFrameThread.joinable())
		grabFrameThread.join();
}

void KnightCamera::grabFrame() {
	int frameTimeout = 0;
	while(runThread) {
		
		frameTimeout = 0;
		while(cap->grab() && frameTimeout++ < 1) {
			//cout << "Grabbing Frame" << endl;
		}
		//cout << "Got frame" << endl;
		_mutex.lock();
		cap->retrieve(frameCapture, 0);
		_mutex.unlock();
	}
}

Mat KnightCamera::getLatestFrame() {
	Mat tmp;
	_mutex.lock();
	frameCapture.copyTo(tmp);
	_mutex.unlock();
	return tmp;
}