#ifndef SRC_SUBSYSTEMS_INTAKESUBSYSTEM_H_
#define SRC_SUBSYSTEMS_INTAKESUBSYSTEM_H_

#include "Utilities/CustomSubsystem.h"
#include "Utilities/Controllers.h"
#include "Utilities/GlobalDefines.h"
#include "WPILib.h"
#include "CANTalon.h"
#include <time.h>
#include <thread>
#include <vector>
#include <iostream>

#define MIN_INTAKE_LOOP_TIME 20

using namespace std;
using namespace frc;

class IntakeSubsystem: public CustomSubsystem {
public:
	IntakeSubsystem(Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector);
	~IntakeSubsystem();

	void init() override;
	void start() override;
	bool isIntakeOut();
	bool isIntakeLocked();
	void setIntakeSpeed(double speed);
	void setIntakePosition(bool out);
	void setIntakeLock(bool lock);
	void subsystemHome() override;
	void stop() override;
private:
	double intakeThreadControlStart, intakeThreadControlEnd;
	int intakeThreadControlElapsedTimeMS;

	DriverStation *ds;

	CANTalon *intakeMotor;
	CANTalon *intakeMotor2;

	DoubleSolenoid *intakeSol;
	DoubleSolenoid *releaseSol;

	double intakeSpeed;

	bool runThread;
	thread intakeThread;

	bool climberDeployed;

	mutex _climberLockMutex;
	mutex _intakePositionMutex;

	void runIntake();
};

#endif /* SRC_SUBSYSTEMS_INTAKESUBSYSTEM_H_ */
