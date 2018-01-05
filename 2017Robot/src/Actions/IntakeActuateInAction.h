/*
 * IntakeActuateInAction.h
 *
 *  Created on: Mar 10, 2017
 *      Author: chris
 */

#ifndef SRC_ACTIONS_INTAKEACTUATEINACTION_H_
#define SRC_ACTIONS_INTAKEACTUATEINACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/IntakeSubsystem.h>

class IntakeActuateInAction: public CustomAction {
public:
	IntakeActuateInAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
	}
	~IntakeActuateInAction() {}

	void start() override {
		intakeSubsystem->setIntakePosition(false);
	};
	void stop() override {};

protected:
	void run() override {};

private:
	IntakeSubsystem *intakeSubsystem;
};

#endif /* SRC_ACTIONS_INTAKEACTUATEINACTION_H_ */
