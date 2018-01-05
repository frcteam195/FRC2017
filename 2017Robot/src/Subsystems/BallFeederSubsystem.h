#ifndef SRC_SUBSYSTEMS_BALLFEEDERSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_BALLFEEDERSUBSYSTEM_H_

#include "Utilities/CustomSubsystem.h"
#include "Utilities/Controllers.h"
#include "Utilities/GlobalDefines.h"
#include "DriveBaseSubsystem.h"
#include "ShooterSubsystem.h"
#include "TurretSubsystem.h"
#include "WPILib.h"
#include "CANTalon.h"
#include <vector>
#include <thread>

#define MIN_FEEDER_LOOP_TIME 100

#define MAX_SPEED 1
#define MIN_SPEED -1

using namespace std;
using namespace frc;

class BallFeederSubsystem: public CustomSubsystem {
public:
	BallFeederSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector);
	~BallFeederSubsystem();

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	void setFeeder(double carouselMotorSpeed, double feederMotorSpeed);
	void setFeederOn();
	void setFeederOff();

private:
	double ballFeederThreadControlStart, ballFeederThreadControlEnd;
	int ballFeederThreadControlElapsedTimeMS;

	DriverStation *ds;

	CANTalon *carouselMotor;
	CANTalon *feederMotor;

	double carouselMotorSpeed;
	double feederMotorSpeed;

	bool runThread;
	thread ballFeederThread;

	void runBallFeeder();
};

#endif /* SRC_SUBSYSTEMS_BALLFEEDERSUBSYSTEM_H_ */
