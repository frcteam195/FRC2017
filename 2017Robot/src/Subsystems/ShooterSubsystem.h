#ifndef SRC_SUBSYSTEMS_SHOOTERSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_SHOOTERSUBSYSTEM_H_

#include "Utilities/CustomSubsystem.h"
#include "WPILib.h"
#include "CANTalon.h"
#include "Utilities/Controllers.h"
#include "Utilities/TuneablePID.h"
#include "Utilities/GlobalDefines.h"
#include <thread>
#include <vector>

#define SPEED_THRESHOLD 350

#define MIN_SHOOTER_LOOP_TIME 75

using namespace std;

class ShooterSubsystem: public CustomSubsystem , public TuneablePID {
public:
	static const int UDP_PORT_NUMBER = 5809;

	ShooterSubsystem(Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector);
	~ShooterSubsystem() {};	//Need to declare and define destructor when implementing CustomSubsystem

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	bool isAtSpeed();
	void bumpShooterSpeed(bool increase);
	void bumpHood(bool increase);
	void setSpeed(double shooterVelocity);
	void setHoodPos(double hoodPos);
	void selectShot(ShotSelection shotSelection);
	double getSetpoint();
	double getHoodSetpoint();
private:
	double shooterThreadControlStart, shooterThreadControlEnd;
	int shooterThreadControlElapsedTimeMS;

	DriverStation *ds;

	CANTalon *shooterWheel;
	CANTalon *hoodMotor;

	double shooterVelocity;
	double hoodPos;

	bool runThread;

	thread shooterThread;

	void assignOutput();
};

#endif /* SRC_SUBSYSTEMS_SHOOTERSUBSYSTEM_H_ */
