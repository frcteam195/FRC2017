/*
 * IntakeInAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_INTAKEINACTION_H_
#define SRC_ACTIONS_INTAKEINACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/IntakeSubsystem.h>

class IntakeInAction: public CustomAction {
public:
	IntakeInAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
	}
	~IntakeInAction() {}

	void start() override {
		intakeSubsystem->setIntakeSpeed(1);
	};
	void stop() override {
		intakeSubsystem->setIntakeSpeed(0);
	};

protected:
	void run() override {};

private:
	IntakeSubsystem *intakeSubsystem;
};

#endif /* SRC_ACTIONS_INTAKEINACTION_H_ */
