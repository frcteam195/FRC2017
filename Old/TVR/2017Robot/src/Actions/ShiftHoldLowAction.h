/*
 * ShiftHoldLowAction.h
 *
 *  Created on: Mar 10, 2017
 *      Author: chris
 */

#ifndef SRC_ACTIONS_SHIFTHOLDLOWACTION_H_
#define SRC_ACTIONS_SHIFTHOLDLOWACTION_H_

#include <Utilities/CustomAction.h>
#include "Subsystems/DriveBaseSubsystem.h"

class ShiftHoldLowAction: public CustomAction {
public:
	ShiftHoldLowAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector):CustomAction(robotControllers, subsystemVector) {
		driveBaseSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()));
	};
	~ShiftHoldLowAction() {};

	void start() override {};
	void start(bool holdLow) {
		driveBaseSubsystem->setHoldLowGear(holdLow);
	};
	void stop() override {};

protected:
	void run() override {};

private:
	DriveBaseSubsystem *driveBaseSubsystem;
};



#endif /* SRC_ACTIONS_SHIFTHOLDLOWACTION_H_ */
