/*
 * ClimbAction.h
 *
 *  Created on: Mar 12, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_CLIMBACTION_H_
#define SRC_ACTIONS_CLIMBACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/IntakeSubsystem.h>

class ClimbAction: public CustomAction {
public:
	ClimbAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
	}
	~ClimbAction() {}

	void start() override {
		if (intakeSubsystem->isIntakeOut()) {
			intakeSubsystem->setIntakePosition(false);
			 this_thread::sleep_for(chrono::milliseconds(1100));
		 }

		 if (!intakeSubsystem->isIntakeLocked()) {
			 intakeSubsystem->setIntakeLock(true);
			 this_thread::sleep_for(chrono::milliseconds(350));
		 }

		 intakeSubsystem->setIntakeSpeed(-1);
	};
	void stop() override {
		intakeSubsystem->setIntakeSpeed(0);
	};

protected:
	void run() override {};

private:
	IntakeSubsystem *intakeSubsystem;
};

#endif /* SRC_ACTIONS_CLIMBACTION_H_ */
