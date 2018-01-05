/*
 * GearPlaceAction.h
 *
 *  Created on: Mar 10, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_GEARPLACEACTION_H_
#define SRC_ACTIONS_GEARPLACEACTION_H_

#include <Utilities/CustomAction.h>
#include "Subsystems/GearSubsystem.h"
#include "Subsystems/DriveBaseSubsystem.h"

class GearPlaceAction: public CustomAction {
public:
	GearPlaceAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector): CustomAction(robotControllers, subsystemVector) {
		gearSubsystem = dynamic_cast<GearSubsystem*>(subsystemVector->at(robotControllers->getGearSubsystemIndex()));
		driveBaseSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()));
	};

	~GearPlaceAction() {}

	void start() override {
		if (_actionMutex.try_lock()) {
			running = true;
			runAction = thread(&GearPlaceAction::run, this);
		}
	};

	void stop() override {};

protected:
	void run() override {
		gearSubsystem->setGearClamp(true);
		driveBaseSubsystem->setDriveSpeed(0, 0);
		driveBaseSubsystem->setGear(false);
		this_thread::sleep_for(chrono::milliseconds(50));
		driveBaseSubsystem->setDriveSpeed(0, 0);
		gearSubsystem->setGearActuatorPos(POSITION_UP);
		gearSubsystem->setGearPusher(true);
		this_thread::sleep_for(chrono::milliseconds(225));
		driveBaseSubsystem->setDriveSpeed(1, -1);
		this_thread::sleep_for(chrono::milliseconds(250));
		gearSubsystem->setGearActuatorPos(POSITION_ZERO);
		this_thread::sleep_for(chrono::milliseconds(100));
		driveBaseSubsystem->setDriveSpeed(0, 0);
		gearSubsystem->setGearPusher(false);
		gearSubsystem->setGearClamp(false);
		//driveBaseSubsystem->setGear(true);
		this_thread::sleep_for(chrono::milliseconds(250));
		running = false;
		runAction.detach();
		_actionMutex.unlock();
	};

private:
	GearSubsystem *gearSubsystem;
	DriveBaseSubsystem *driveBaseSubsystem;

};

#endif /* SRC_ACTIONS_GEARPLACEACTION_H_ */
