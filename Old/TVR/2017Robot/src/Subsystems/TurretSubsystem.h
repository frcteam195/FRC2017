#ifndef SRC_SUBSYSTEMS_TURRETSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_TURRETSUBSYSTEM_H_

#include "Utilities/CustomSubsystem.h"
#include "Utilities/TuneablePID.h"
#include "Subsystems/VisionReceiverSubsystem.h"
#include "Utilities/Controllers.h"
#include "Utilities/GlobalDefines.h"
#include "DriveBaseSubsystem.h"
#include "WPILib.h"
#include "CANTalon.h"
#include "AHRS.h"
#include <vector>
#include <thread>
#include <iostream>

#define MIN_TURRET_LOOP_TIME 4
#define TURRET_HOME_TIMEOUT_MS 2000

using namespace std;


enum TurretControlMode { vision, manualSetpoint, predictiveSeek, raw };

class TurretSubsystem: public CustomSubsystem, public TuneablePID {
public:
	TurretSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector);
	~TurretSubsystem();

	static const int UDP_PORT_NUMBER = 5806;

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;
	double getTurretPos();
	double getTurretSetpoint();
	void setTurretPos(double turretPos);
	void selectShot(ShotSelection shotSelection);
	void configVelocityAccel(double velocity, double accel);

	void homeTurret();

	double sgn(double x) {
		return (x > 0) - (x < 0);
	};


	bool isOnTarget();

private:
	double turretThreadControlStart, turretThreadControlEnd;
	int turretThreadControlElapsedTimeMS;

	double timeoutStart, timeoutEnd;
	int timeoutElapsedTimeMS;

	DriverStation *ds;

	CANTalon *turretMotor;

	DigitalInput *turretSwitch;

	bool runThread;
	bool homing;

	double turretPos;

	thread turretSubsystemThread;

	void runTurret();

	double prevVelocity;
	double prevAccel;
};

#endif /* SRC_SUBSYSTEMS_TURRETSUBSYSTEM_H_ */
