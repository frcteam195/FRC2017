/*
 * VisionReceiverSubsystem.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: roberthilton
 */

#include <Subsystems/VisionReceiverSubsystem.h>

using namespace std;
using namespace frc;

VisionReceiverSubsystem::VisionReceiverSubsystem(int udpPort, vector<CustomSubsystem *> *subsystemVector) {
	this->udpPort = udpPort;
	subsystemVector->push_back(this);
	recvlenReceive = 0;
	recvlenSend = 0;
	fd = 0;
	addrlen = 0;
	status = 0;
	runThread = false;
	visionData = new VisionData();
	visionDataTemp = new VisionData();

	visionReceiveThreadControlStart = 0;
	visionReceiveThreadControlEnd = 0;
	visionReceiveThreadControlElapsedTimeMS = 0;

	visionSendThreadControlStart = 0;
	visionSendThreadControlEnd = 0;
	visionSendThreadControlElapsedTimeMS = 0;

	enableVision = false;
}

VisionReceiverSubsystem::~VisionReceiverSubsystem() {}

void VisionReceiverSubsystem::init() {
	//cout << "Init start for VisionReceiver!\n";
	addrlen = sizeof(remoteAddr); /* length of addresses */

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		cout << "cannot create socket\n";
		status = 1;
		return;
	}

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 10000;
	if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0)
	    cout << "Error setting socket timeout" << endl;

	int bufferLength = 51*2;

	if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bufferLength, sizeof(bufferLength)) == -1)
		cout << "Error setting socket buffer length" << endl;

	/* bind the socket to any valid IP address and a specific port */

	memset((char *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET;
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(udpPort);

	remoteAddr.sin_family = AF_INET;
	//remoteAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	//remoteAddr.sin_port = htons(0);
	inet_aton("10.1.95.21", &(remoteAddr.sin_addr));
	remoteAddr.sin_port = htons(udpPort);

	if (bind(fd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		cout << "bind failed";
		status = 2;
		return;
	}
	status = 0;
	//cout << "Init finish for VisionReceiver!\n";
}

void VisionReceiverSubsystem::start() {
	if (status != 0)
		return;

	runThread = true;
	udpReceiveThread = thread(&VisionReceiverSubsystem::runUDPReceive, this);
	//udpSendThread = thread(&VisionReceiverSubsystem::runUDPSend, this);
}

void VisionReceiverSubsystem::stop() {
	if (status != 0)
		return;

	runThread = false;
	if (udpReceiveThread.joinable())
		udpReceiveThread.join();

}

void VisionReceiverSubsystem::runUDPReceive() {
	while (runThread)
	{
		visionReceiveThreadControlStart = Timer::GetFPGATimestamp();

		recvlenReceive = recvfrom(fd, buf, BUFSIZE, 0, (sockaddr *) &remoteAddr, &addrlen);
		if (recvlenReceive > 0) {
			buf[recvlenReceive] = 0;
			_mutex.lock();
			visionData = processUDPPacket(buf, recvlenReceive);
			_mutex.unlock();
			//cout << "Angle: " << visionData->angleDeviation << endl << "On Target: " << visionData->onTarget << endl;
		} else
			;//cout << "Vision thread error!\n";


		//Ensure that we do not try to run faster than the sample period
		do {
			visionReceiveThreadControlEnd = Timer::GetFPGATimestamp();
			visionReceiveThreadControlElapsedTimeMS = (int) ((visionReceiveThreadControlEnd - visionReceiveThreadControlStart) * 1000);
			if (visionReceiveThreadControlElapsedTimeMS < VISION_RECEIVE_RATE_MS)
				this_thread::sleep_for(chrono::milliseconds(VISION_RECEIVE_RATE_MS - visionReceiveThreadControlElapsedTimeMS));
		} while(visionReceiveThreadControlElapsedTimeMS < VISION_RECEIVE_RATE_MS);
		//cout << "loop rate: " << elapsedTimeMS << endl;
	}
}

void VisionReceiverSubsystem::runUDPSend() {
	while (runThread)
	{
		visionSendThreadControlStart = Timer::GetFPGATimestamp();

		if (enableVision)
			strEnableVision = "true";
		else
			strEnableVision = "false";

		string sendStr = "VisionEnable:" + strEnableVision +";";
		recvlenSend = sendto(fd, sendStr.c_str(), sendStr.length()+1, 0, (sockaddr *) &remoteAddr, addrlen);
		if (recvlenSend > 0) {
			//cout << "Packet Sent!" << endl;;
		} else
			;

		//Ensure that we do not try to run faster than the sample period
		do {
			visionSendThreadControlEnd = Timer::GetFPGATimestamp();
			visionSendThreadControlElapsedTimeMS = (int) ((visionSendThreadControlEnd - visionSendThreadControlStart) * 1000);
			if (visionSendThreadControlElapsedTimeMS < VISION_SEND_RATE_MS)
				this_thread::sleep_for(chrono::milliseconds(VISION_SEND_RATE_MS - visionSendThreadControlElapsedTimeMS));
		} while(visionSendThreadControlElapsedTimeMS < VISION_SEND_RATE_MS);
		//cout << "loop rate: " << elapsedTimeMS << endl;
	}
}

void VisionReceiverSubsystem::setEnableVision(bool enable) {
	_subsystemMutex.lock();
	enableVision = enable;
	_subsystemMutex.unlock();
}

VisionData *VisionReceiverSubsystem::processUDPPacket(unsigned char* buf, int recvlen) {
	visionDataTemp = new VisionData();

	for (unsigned int i = 0; i < strlen((const char*) buf); ++i)
		buf[i] = tolower(buf[i]);

	vector<char *> vS;
	char* chars_array = strtok((char*) buf, ";");
	while (chars_array) {
		vS.push_back(chars_array);
		chars_array = strtok(NULL, ";");
	}
	for (unsigned int i = 0; i < vS.size(); ++i) {
		char* cmd = strtok((char*) vS[i], ":");
		char* value = strtok(NULL, ":");
		if (strcmp(cmd, "angledeviation") == 0)
			visionDataTemp->angleDeviation = atof(value);
		else if (strcmp(cmd, "targetdistance") == 0)
			visionDataTemp->targetDistance = atof(value);
		else if (strcmp(cmd, "ontarget") == 0) {
			if (strcmp(value, "true") == 0)
				visionDataTemp->onTarget = true;
			else
				visionDataTemp->onTarget = false;
		}
		else if (strcmp(cmd, "targetfound") == 0) {
			if (strcmp(value, "true") == 0)
				visionDataTemp->targetFound = true;
			else
				visionDataTemp->targetFound = false;
		} else if (strcmp(cmd, "sequence") == 0) {
			visionDataTemp->sequence = (unsigned long) atol(value);
		}
	}
	return visionDataTemp;
}

VisionData *VisionReceiverSubsystem::getVisionData() {
	return visionData;
}
