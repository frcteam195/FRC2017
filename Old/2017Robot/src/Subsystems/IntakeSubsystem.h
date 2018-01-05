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

#define MIN_INTAKE_LOOP_TIME 10

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
	void setIntakePosition(bool out);
	void setIntakeLock(bool lock);
	void subsystemHome() override;
	void stop() override;
private:
	double intakeThreadControlStart, intakeThreadControlEnd;
	int intakeThreadControlElapsedTimeMS;

	DriverStation *ds;


	Joystick *buttonBox1;
	Joystick *buttonBox2;

	CANTalon *intakeMotor;
	CANTalon *intakeMotor2;

	DoubleSolenoid *intakeSol;
	DoubleSolenoid *releaseSol;

	bool runThread;
	thread intakeThread;

	bool climberDeployed;

	void runIntake();
};

#endif /* SRC_SUBSYSTEMS_INTAKESUBSYSTEM_H_ */
