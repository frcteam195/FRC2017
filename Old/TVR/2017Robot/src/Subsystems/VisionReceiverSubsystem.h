/*
 * VisionReceiverSubsystem.h
 *
 *  Created on: Jan 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_SUBSYSTEMS_VISIONRECEIVERSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_VISIONRECEIVERSUBSYSTEM_H_

#define BUFSIZE 2048
#define VISION_RECEIVE_RATE_MS 4
#define VISION_SEND_RATE_MS 200

#include "Utilities/CustomSubsystem.h"
#include "Utilities/VisionData.h"

#include "WPILib.h"

#include <arpa/inet.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <mutex>
#include <netdb.h>
#include <sys/socket.h>
#include <cstring>
#include <time.h>
#include <thread>
#include <vector>

using namespace std;

class VisionReceiverSubsystem: public CustomSubsystem {
private:
	double visionSendThreadControlStart, visionSendThreadControlEnd;
	int visionSendThreadControlElapsedTimeMS;

	double visionReceiveThreadControlStart, visionReceiveThreadControlEnd;
	int visionReceiveThreadControlElapsedTimeMS;

	sockaddr_in localAddr; /* our address */
	sockaddr_in remoteAddr; /* remote address */
	socklen_t addrlen;

	int recvlenReceive;
	int recvlenSend;
	int fd;
	unsigned char buf[BUFSIZE];

	int status;

	bool runThread;
	bool enableVision;
	string strEnableVision;

	int udpPort;

	thread udpReceiveThread;
	thread udpSendThread;

	VisionData *visionData;
	VisionData *visionDataTemp;
	mutex _mutex;

	void runUDPReceive();
	void runUDPSend();
	VisionData *processUDPPacket(unsigned char* buf, int recvlen);

public:
	VisionReceiverSubsystem(int udpPort, vector<CustomSubsystem *> *subsystemVector);
	~VisionReceiverSubsystem();

	void init() override;
	void start() override;
	void stop() override;
	void subsystemHome() override {};

	void setEnableVision(bool enable);

	VisionData *getVisionData();
};

#endif /* SRC_SUBSYSTEMS_VISIONRECEIVERSUBSYSTEM_H_ */
