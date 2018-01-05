/*
 * GearPickupAction.h
 *
 *  Created on: Mar 10, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_GEARPICKUPACTION_H_
#define SRC_ACTIONS_GEARPICKUPACTION_H_

#include <Utilities/CustomAction.h>
#include "Subsystems/GearSubsystem.h"

#define PICKUP_VEL 50
#define PICKUP_ACCEL 105

#define DOWN_VEL 200
#define DOWN_ACCEL 800

class GearPickupAction: public CustomAction {
public:
	GearPickupAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector): CustomAction(robotControllers, subsystemVector) {
		gearSubsystem =  dynamic_cast<GearSubsystem*>(subsystemVector->at(robotControllers->getGearSubsystemIndex()));
	}

	~GearPickupAction() {}

	void start() override {
		if (_actionMutex.try_lock()) {
			running = true;
			runAction = thread(&GearPickupAction::run, this);
		}
	};

	void stop() override {};

protected:
	void run() override {
		gearSubsystem->configVelocityAccel(DOWN_VEL, DOWN_ACCEL);
		gearSubsystem->setGearActuatorPos(POSITION_ZERO);
		gearSubsystem->setGearClamp(true);
		this_thread::sleep_for(chrono::milliseconds(75));
		gearSubsystem->setGearActuatorPos(POSITION_DOWN);

		timeoutElapsedTimeMS = 0;

		timeoutStart = Timer::GetFPGATimestamp();

		while (abs(gearSubsystem->getGearActuatorPos() - POSITION_DOWN) > DEVIATION_THRESHOLD && timeoutElapsedTimeMS < GEAR_DOWN_ACTION_TIMEOUT_MS) {
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}

		gearSubsystem->configVelocityAccel(PICKUP_VEL, PICKUP_ACCEL);

		while (robotControllers->getDriveJoystick()->GetRawButton(DRIVER_PICKUP_GEAR) || robotControllers->getButtonBox1()->GetRawButton(PICKUP_GEAR)) {
			this_thread::sleep_for(chrono::milliseconds(25));
		}

		this_thread::sleep_for(chrono::milliseconds(350));

		gearSubsystem->setGearClamp(false);

		this_thread::sleep_for(chrono::milliseconds(75));

		gearSubsystem->setGearActuatorPos(POSITION_HOLD);
		running = false;
		runAction.detach();
		_actionMutex.unlock();
	};

private:
	GearSubsystem *gearSubsystem;
};

#endif /* SRC_ACTIONS_GEARPICKUPACTION_H_ */
