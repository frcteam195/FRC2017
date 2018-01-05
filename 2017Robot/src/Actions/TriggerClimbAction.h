/*
 * TriggerClimbAction.h
 *
 *  Created on: May 13, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_TRIGGERCLIMBACTION_H_
#define SRC_ACTIONS_TRIGGERCLIMBACTION_H_


#include <Utilities/CustomAction.h>
#include <Subsystems/IntakeSubsystem.h>

class TriggerClimbAction: public CustomAction {
public:
	TriggerClimbAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
	}
	~TriggerClimbAction() {}

	void start() override {
		if (intakeSubsystem->isIntakeLocked()) {
			intakeSubsystem->setIntakeSpeed(-1);
		}
	};
	void stop() override {};

protected:
	void run() override {};

private:
	IntakeSubsystem *intakeSubsystem;
};


#endif /* SRC_ACTIONS_TRIGGERCLIMBACTION_H_ */
