/*
 * ManualTurretControlAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_MANUALTURRETCONTROLACTION_H_
#define SRC_ACTIONS_MANUALTURRETCONTROLACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/TurretSubsystem.h>

#define MANUAL_TIME_STEP_MS 20
#define MANUAL_CONTROL_STEP 0.1
#define TURRET_MANUAL_VEL 1200
#define TURRET_MANUAL_ACCEL 2800

class ManualTurretControlAction: public CustomAction {
public:
	ManualTurretControlAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));

		axisControlStart = 0;
		axisControlEnd = 0;
		axisControlElapsedTimeMS = 0;
		zAxis = 0;
	}
	~ManualTurretControlAction() {}

	void start() override {
		axisControlStart = Timer::GetFPGATimestamp();
		turretSubsystem->configVelocityAccel(TURRET_MANUAL_VEL, TURRET_MANUAL_ACCEL);
		zAxis = robotControllers->getArmJoystick()->GetRawAxis(MANUAL_CONTROL_AXIS);
		zAxis = abs(zAxis) < TURRET_JOYSTICK_DEADBAND ? 0 : zAxis;
		if (zAxis != 0) {
			zAxis = turretSubsystem->sgn(zAxis) * ((abs(zAxis) - TURRET_JOYSTICK_DEADBAND) / (1 - TURRET_JOYSTICK_DEADBAND));
			zAxis *= MANUAL_CONTROL_STEP;
			turretSubsystem->setTurretPos(turretSubsystem->getTurretPos() + zAxis);
		}

		do {
			axisControlEnd = Timer::GetFPGATimestamp();
			axisControlElapsedTimeMS = (int) ((axisControlEnd - axisControlStart) * 1000);
			if (axisControlElapsedTimeMS < MANUAL_TIME_STEP_MS)
				this_thread::sleep_for(chrono::milliseconds(MANUAL_TIME_STEP_MS - axisControlElapsedTimeMS));
		} while(axisControlElapsedTimeMS < MANUAL_TIME_STEP_MS);
	};
	void stop() override {

	};

protected:
	void run() override {};

private:
	double zAxis;
	double axisControlStart, axisControlEnd;
	int axisControlElapsedTimeMS;

	TurretSubsystem *turretSubsystem;
};

#endif /* SRC_ACTIONS_MANUALTURRETCONTROLACTION_H_ */
