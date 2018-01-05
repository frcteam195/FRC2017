#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <thread>
#include <chrono>
#include <signal.h>

using namespace std;

bool visionRunning = false;
bool ethernetUp = true;
bool pingReceived = true;
int ethernetDownCounter = 0;
//const char* rebootCommand = "shutdown -r now";	//IntelJoule
const char* rebootCommand = "reboot"; //Jetson
//const char* startVisionService = "systemctl start knight-vision-exec.service";	//IntelJouleServiceCommand
const char* startVisionService = "nohup /home/ubuntu/KnightVision/KnightVision >> /dev/null &";		//JetsonCommand
const char* getNetworkInfoCommand = "ifconfig";
const char* getProcessInfoCommand = "ps aux | grep Knight";
const char* visionSearchString = "Vision";
const char* searchIP = "10.1.95.21";
const char* pingCommand = "ping -c 1 -t 1 10.1.95.22";
const char* pingResponse = "64 bytes from 10.1.95.22";
const char* listUSBDevices = "lsusb | grep ARC";
const char* cameraName = "ARC";
const char* resetSuccessful = "Reset successful";
const char* killVision = "killall KnightVision";
int pingCounter = 0;
int cameraResetCounter = 0;
string s;

string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != NULL)
            result += buffer.data();
    }
    return result;
}

bool checkVisionRunning() {
	s = exec(getProcessInfoCommand);
	return s.find(visionSearchString) != string::npos;
}

bool checkEthernetUp() {
	s = exec(getNetworkInfoCommand);
	return s.find(searchIP) != string::npos;
}

bool checkPing() {
	s = exec(pingCommand);
	return s.find(pingResponse) != string::npos;
}

bool resetUSBCamera() {
	s = exec(listUSBDevices);
	if (s.find(cameraName) != string::npos) {
		string busNum = s.substr(4,3);
		string deviceNum = s.substr(15,3);
		s = exec(("/usr/local/bin/usbreset /dev/bus/usb/" + busNum + "/" + deviceNum).c_str());
		return s.find(resetSuccessful) != string::npos;
	}
	return false;
	
}

void interruptHandler(int s){
	//printf("Caught signal %d\n",s);
	if (s == 2) {
		system(killVision);
		exit(1);
	}
}

int main() {
	//Handle Ctrl-C Signals
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = interruptHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
	//Wait to enable process monitor to ensure radio and network stack are finished loading
	this_thread::sleep_for(chrono::milliseconds(30000));
	while (true) {
		visionRunning = checkVisionRunning();
		ethernetUp = checkEthernetUp();
		pingReceived = checkPing();
		
		
		if (!visionRunning) {
			
			if (resetUSBCamera()) {
				cout << "Camera reset!" << endl;
				this_thread::sleep_for(chrono::milliseconds(1000));
				cameraResetCounter = 0;
				system(startVisionService);
				cout << "Restarting vision service!" << endl;
			} else if (cameraResetCounter++ > 2) {
				cout << "Rebooting Jetson!" << endl;
				system(rebootCommand);
			} else {
				cout << "Camera reset failed!" << endl;
			}
				
			this_thread::sleep_for(chrono::milliseconds(2000));
		}
		
		
		if (!ethernetUp) {
			
			if (ethernetDownCounter++ > 2) {
				cout << "Rebooting Jetson!" << endl;
				system(rebootCommand);
			}
				
			this_thread::sleep_for(chrono::milliseconds(250));
		} else
			ethernetDownCounter = 0;
			
			
		if (!pingReceived) {
			
			if (pingCounter++ > 1) {
				cout << "Rebooting Jetson!" << endl;
				system(rebootCommand);
			}
				
			this_thread::sleep_for(chrono::milliseconds(250));
		} else
			ethernetDownCounter = 0;
	
	
		this_thread::sleep_for(chrono::milliseconds(500));
	}
	return 0;
}