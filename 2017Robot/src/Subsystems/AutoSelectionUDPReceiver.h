/*
 * AutoSelectionUDPReceiver.h
 *
 *  Created on: Mar 16, 2017
 *      Author: roberthilton
 */

#ifndef SRC_SUBSYSTEMS_AUTOSELECTIONUDPRECEIVER_H_
#define SRC_SUBSYSTEMS_AUTOSELECTIONUDPRECEIVER_H_


#define BUFSIZE 2048
#define AUTO_UDP_RECEIVE_RATE_MS 200

#include "Utilities/Controllers.h"

#include <arpa/inet.h>
#include <iostream>
#include <mutex>
#include <netdb.h>
#include <sys/socket.h>
#include <time.h>
#include <cstring>
#include <thread>
#include <vector>

using namespace std;

class AutoSelectionUDPReceiver: public CustomSubsystem {
public:
	AutoSelectionUDPReceiver(int portNumber, vector<CustomSubsystem *> *subsystemVector) {
		subsystemVector->push_back(this);

		this->portNumber = portNumber;
		autonomous = AutoSelection::kHopper;
		recvlen = 0;
		fd = 0;
		status = 0;
		runThread = false;
		addrlen = 0;

		timeoutStart = 0;
		timeoutEnd = 0;
		timeoutElapsedTimeMS = 0;
	};

	void init() override {
		addrlen = sizeof(remoteAddr); /* length of addresses */

		if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			status = 1;
			return;
		}

		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 100000;
		if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0)
		    cout << "Error setting socket timeout" << endl;

		memset((char *) &localAddr, 0, sizeof(localAddr));
		localAddr.sin_family = AF_INET;
		localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		localAddr.sin_port = htons(portNumber);

		remoteAddr.sin_family = AF_INET;
		remoteAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		remoteAddr.sin_port = htons(portNumber);

		if (bind(fd, (sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
			status = 2;
			return;
		}
		status = 0;
	};

	void subsystemHome() override {};

	void start() override {
		runThread = true;

		udpReceiveThread = thread(&AutoSelectionUDPReceiver::runUDPReceive, this);
	};

	void stop() override {
		runThread = false;
	};

	AutoSelection getAutoMode() {
		return autonomous;
	};
private:
	uint16_t portNumber;
	sockaddr_in localAddr; /* our address */
	sockaddr_in remoteAddr; /* remote address */
	socklen_t addrlen;

	int recvlen;
	int fd;
	unsigned char buf[BUFSIZE];

	int status;

	thread udpReceiveThread;

	bool runThread;
	mutex _mutex;

	double timeoutStart, timeoutEnd;
	int timeoutElapsedTimeMS;

	AutoSelection autonomous;

	AutoSelection processUDPPacket(unsigned char* buf, int recvlen) {
		AutoSelection autonomous = AutoSelection::kHopper;
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
			if (strcmp(cmd, "autonomous") == 0)
				switch (atoi(value)) {
				case 0:
					autonomous = AutoSelection::kHopper;
					break;
				case 1:
					autonomous = AutoSelection::kSideGearHopper;
					break;
				case 2:
					autonomous = AutoSelection::kSideGear;
					break;
				case 3:
					autonomous = AutoSelection::kCenterGear;
					break;
				case 4:
					autonomous = AutoSelection::kFarHopper;
					break;
				default:
					autonomous = AutoSelection::kHopper;
					break;
				}
		}
		return autonomous;
	};

	void runUDPReceive() {
		while (runThread) {
			timeoutStart = Timer::GetFPGATimestamp();

			recvlen = recvfrom(fd, buf, BUFSIZE, 0, (sockaddr *) &remoteAddr, &addrlen);
			if (recvlen > 0) {
				buf[recvlen] = 0;
				_mutex.lock();
				autonomous = processUDPPacket(buf, recvlen);
				_mutex.unlock();

			//cout << autonomous << endl;

			} else
				;//cout << "uh oh - something went wrong!\n";

			do {
				timeoutEnd = Timer::GetFPGATimestamp();
				timeoutElapsedTimeMS = (int) ((timeoutEnd - timeoutStart) * 1000);
				if (timeoutElapsedTimeMS < AUTO_UDP_RECEIVE_RATE_MS)
					this_thread::sleep_for(chrono::milliseconds(AUTO_UDP_RECEIVE_RATE_MS - timeoutElapsedTimeMS));
			} while(timeoutElapsedTimeMS < AUTO_UDP_RECEIVE_RATE_MS);
		}
		udpReceiveThread.detach();
	};
};


#endif /* SRC_SUBSYSTEMS_AUTOSELECTIONUDPRECEIVER_H_ */
