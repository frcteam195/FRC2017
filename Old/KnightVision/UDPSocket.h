#ifndef KNIGHTVISION_UDPSOCKET_H_
#define KNIGHTVISION_UDPSOCKET_H_

#define BUFSIZE 2048
#define PID_RECEIVE_RATE_MS 40
#define PID_SEND_RATE_MS 10

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

class UDPSocket {
public:
	UDPSocket(int portNumber);
	~UDPSocket() {}

	void init();
	void start();
	void stop();

	void updateValues(double deviation, bool onTarget);
	void udpSendSynchronous(double deviation, bool onTarget);
private:
	double deviation;
	string onTarget;

	uint16_t portNumber;
	sockaddr_in localAddr; /* our address */
	sockaddr_in remoteAddr; /* remote address */
	socklen_t addrlen;

	int recvlen;
	int fd;
	unsigned char buf[BUFSIZE];

	int status;

	thread udpReceiveThread;
	thread udpSendThread;

	bool autoUpdate;
	bool autoUpdateSetpoint;
	bool runThread;
	mutex _mutex;

	void runUDPReceive();
	void runUDPSend();
};

#endif /* KNIGHTVISION_UDPSOCKET_H_ */
