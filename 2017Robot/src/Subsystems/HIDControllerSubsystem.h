/*
 * HIDControllerSubsystem.h
 *
 *  Created on: Mar 9, 2017
 *      Author: roberthilton
 */

#ifndef SRC_SUBSYSTEMS_HIDCONTROLLERSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_HIDCONTROLLERSUBSYSTEM_H_

#include "WPILib.h"
#include "Utilities/CustomSubsystem.h"
#include "Utilities/Controllers.h"
#include "Utilities/KnightJoystick.h"
#include "Actions/ActionIncludes.h"
#include <thread>

#define MIN_HID_THREAD_LOOP_TIME_MS 45

using namespace frc;
using namespace std;

class HIDControllerSubsystem: public CustomSubsystem {
public:
	HIDControllerSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector);
	~HIDControllerSubsystem() {}

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

private:
	vector<CustomSubsystem*> *subsystemVector;

	ClimbAction *climbAction;
	AgitateAction *agitateAction;
	GearOscillatorAction *gearOscillatorAction;
	GearPickupAction *gearPickupAction;
	GearPlaceAction *gearPlaceAction;
	HomeGearAction *homeGearAction;
	HomeTurretAction *homeTurretAction;
	IntakeActuateOutAction *intakeActuateOutAction;
	IntakeActuateInAction *intakeActuateInAction;
	IntakeInAction *intakeInAction;
	IntakeOutAction *intakeOutAction;
	ManualTurretControlAction *manualTurretControlAction;
	RunBallFeederAction *runBallFeederAction;
	ShiftHoldLowAction *shiftHoldLowAction;
	ShiftAction *shiftAction;
	ShooterAction *shooterAction;
	TargetSearchAction *targetSearchAction;
	TriggerClimbAction *triggerClimbAction;
	VisionTurretControlAction *visionTurretControlAction;

	DriverStation *ds;

	KnightJoystick *driveJoystick;
	KnightJoystick *armJoystick;
	KnightJoystick *buttonBox1;
	KnightJoystick *buttonBox2;

	bool runThread;

	bool comingFromAuto;

	Controllers *robotControllers;

	double x, y, absLeft, absRight,normalLeft,normalRight, left, right;

	thread driveJoystickThread;
	double driveJoystickThreadControlStart, driveJoystickThreadControlEnd;
	int driveJoystickThreadControlElapsedTimeMS;

	int gearButtonCount;

	thread armJoystickThread;
	double armJoystickThreadControlStart, armJoystickThreadControlEnd;
	int armJoystickThreadControlElapsedTimeMS;

	thread buttonBoxThread;
	double buttonBoxThreadControlStart, buttonBoxThreadControlEnd;
	int buttonBoxThreadControlElapsedTimeMS;

	void runDriveJoystickThread();
	void runArmJoystickThread();
	void runButtonBoxThread();

	double sgn(double x) {
		return (x > 0) - (x < 0);
	};

};

#endif /* SRC_SUBSYSTEMS_HIDCONTROLLERSUBSYSTEM_H_ */
