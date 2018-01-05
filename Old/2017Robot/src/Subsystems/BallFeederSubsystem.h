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

#define MIN_FEEDER_LOOP_TIME 10

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
private:
	double ballFeederThreadControlStart, ballFeederThreadControlEnd;
	int ballFeederThreadControlElapsedTimeMS;

	DriveBaseSubsystem *driveSubsystem;
	TurretSubsystem *turretSubsystem;

	DriverStation *ds;
	ShooterSubsystem *shooterSubsystem;

	Joystick *buttonBox1;
	Joystick *buttonBox2;

	CANTalon *carouselMotor;
	CANTalon *feederMotor;

	bool runThread;
	thread ballFeederThread;

	void runBallFeeder();
};

#endif /* SRC_SUBSYSTEMS_BALLFEEDERSUBSYSTEM_H_ */
