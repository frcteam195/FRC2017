/*
 * ShiftAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: chris
 */

#ifndef SRC_ACTIONS_SHIFTACTION_H_
#define SRC_ACTIONS_SHIFTACTION_H_


#include <Utilities/CustomAction.h>
#include "Subsystems/DriveBaseSubsystem.h"

class ShiftAction: public CustomAction {
public:
	ShiftAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector):CustomAction(robotControllers, subsystemVector) {
		driveBaseSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()));
	};
	~ShiftAction() {};

	void start() override {};
	void start(bool highGear) {
		driveBaseSubsystem->setGear(highGear);
	};
	void stop() override {};

protected:
	void run() override {};

private:
	DriveBaseSubsystem *driveBaseSubsystem;
};



#endif /* SRC_ACTIONS_SHIFTACTION_H_ */
