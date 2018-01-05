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

#define MANUAL_TIME_STEP_MS 20
#define MANUAL_CONTROL_STEP 0.1

#define MIN_TURRET_LOOP_TIME 10

using namespace std;


enum TurretControlMode { vision, manualSetpoint, predictiveSeek, raw };

class TurretSubsystem: public CustomSubsystem, public TuneablePID {
public:
	TurretSubsystem(VisionReceiverSubsystem *visionReciever, Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector);
	~TurretSubsystem();

	static const int UDP_PORT_NUMBER = 5806;

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	bool isOnTarget();

private:
	double turretThreadControlStart, turretThreadControlEnd;
	int turretThreadControlElapsedTimeMS;

	double axisControlStart, axisControlEnd;
	int axisControlElapsedTimeMS;

	bool filteredOnTarget;
	int filterCounter;
	bool prevOnTargetVal;

	DriverStation *ds;

	DriveBaseSubsystem *driveSubsystem;

	VisionReceiverSubsystem *visionReceiver;

	VisionData *visionData;

	Joystick *armJoystick;
	Joystick *buttonBox1;

	CANTalon *turretMotor;

	TurretControlMode ctrlMode;

	bool runThread;

	thread turretSubsystemThread;

	double deviation;

	void runTurret();
	double sgn(double x);

	double zAxis;
};

#endif /* SRC_SUBSYSTEMS_TURRETSUBSYSTEM_H_ */
