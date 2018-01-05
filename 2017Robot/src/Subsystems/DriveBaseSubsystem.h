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

#define MIN_DRIVE_LOOP_TIME 10
#define MIN_SHIFT_LOOP_TIME 50

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

	double getAveragePosition();
	double getLeftDrivePosition();
	double getRightDrivePosition();

	void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed);
	void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed, bool slowDown);
	void setHoldLowGear(bool holdLowGear);
	void setGear(bool highGear);
	void setPosition(double position);
	bool isHighGear();

	CANTalon::MotionProfileStatus getLeftMPStatus();
	CANTalon::MotionProfileStatus getRightMPStatus();

	void processMotionProfile(vector<vector< double> *> *mpLeftBuffer, vector<vector< double> *> *mpRightBuffer);
	void changeControlMode(CANSpeedController::ControlMode controlMode);
	void changeTalonControlMode(CANTalon::TalonControlMode controlMode);

	void setDrivePID(double kP, double kI, double kD, double ff, int profileNum);
	bool isPositionWithinRange(double range);

	void setMotionMagicVelocityAccel(double vel, double accel);

private:
	void processMPLeft();
	void processMPRight();
	void processMP(CANTalon *talonSRX, vector<vector< double> *> *mpBuffer);

	double leftDriveThreadControlStart, leftDriveThreadControlEnd;
	int leftDriveThreadControlElapsedTimeMS;

	double rightDriveThreadControlStart, rightDriveThreadControlEnd;
	int rightDriveThreadControlElapsedTimeMS;

	double shiftThreadControlStart, shiftThreadControlEnd;
	int shiftThreadControlElapsedTimeMS;

	double sgn(double x);
	DriverStation *ds;

	double leftDriveSpeed;
	double rightDriveSpeed;
	bool highGear, holdLow;
	vector<vector< double> *> *mpLeftBuffer;
	vector<vector< double> *> *mpRightBuffer;

	bool requestSetLeftPosition;
	bool requestSetRightPosition;
	double position;

	CANTalon::MotionProfileStatus mpStatusLeft;
	CANTalon::MotionProfileStatus mpStatusRight;
	CANTalon *leftDrive;
	CANTalon *leftDriveSlave1;
	CANTalon *leftDriveSlave2;
	CANTalon *rightDrive;
	CANTalon *rightDriveSlave1;
	CANTalon *rightDriveSlave2;

	CANSpeedController::ControlMode driveControlMode;

	AHRS *navX;

	DoubleSolenoid *shiftSol;

	thread leftDriveThread;
	thread rightDriveThread;
	thread shiftThread;

	thread leftMPBufferProcess;
	thread rightMPBufferProcess;

	mutex shiftMutex;
	mutex holdLowMutex;

	bool runThread;

	void runLeftDrive();
	void runRightDrive();
	void shift();
};

#endif /* SRC_SUBSYSTEMS_DRIVEBASESUBSYSTEM_H_ */
