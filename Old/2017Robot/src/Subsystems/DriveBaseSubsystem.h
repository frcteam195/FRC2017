#ifndef SRC_SUBSYSTEMS_DRIVEBASESUBSYSTEM_H_
#define SRC_SUBSYSTEMS_DRIVEBASESUBSYSTEM_H_

#include "Utilities/Controllers.h"
#include "Utilities/GlobalDefines.h"
#include "Utilities/CustomSubsystem.h"
#include "WPILib.h"
#include "CANTalon.h"
#include "Utilities/KnightPIDController.h"
#include "Utilities/DriveMotorValues.h"
#include "AHRS.h"
#include <thread>
#include <vector>
#include <iostream>

#define MIN_DRIVE_LOOP_TIME 5
#define MIN_SHIFT_LOOP_TIME 10

#define STEP_ONE_BLUE 38
#define STEP_ONE_RED 33
#define TURN_ANGLE_ONE_RED 85
#define TURN_ANGLE_ONE_BLUE 85
#define STEP_TWO 27

#define DRIVE_JOYSTICK_DEADBAND 0.1

using namespace std;
using namespace frc;

class DriveBaseSubsystem : public CustomSubsystem {
public:
	DriveBaseSubsystem(Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector);
	~DriveBaseSubsystem();

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	bool isAutoDriveFinished();

	bool isHighGear();
private:
	double driveThreadControlStart, driveThreadControlEnd;
	int driveThreadControlElapsedTimeMS;

	double shiftThreadControlStart, shiftThreadControlEnd;
	int shiftThreadControlElapsedTimeMS;

	bool autoDriveFinished;
	double sgn(double x);
	DriverStation *ds;

	Joystick *driveJoystick;

	CANTalon *leftDrive;
	CANTalon *leftDriveSlave1;
	CANTalon *leftDriveSlave2;
	CANTalon *rightDrive;
	CANTalon *rightDriveSlave1;
	CANTalon *rightDriveSlave2;

	AHRS *navX;

	KnightPIDController *angleController;

	DoubleSolenoid *shiftSol;

	thread driveThread;
	thread shiftThread;

	double currentValTmp;
	double avgPosTmp;

	double x, y, left, right, absLeft, absRight, normalLeft, normalRight;
	const double MAX_OUTPUT = 1;

	DoubleSolenoid::Value driveGear;
	bool highGear, holdLow;

	bool runThread;

	void drive();
	void shift();
	DriveMotorValues driveAtAngle(double angle, DriveMotorValues values);
};

#endif /* SRC_SUBSYSTEMS_DRIVEBASESUBSYSTEM_H_ */
