/*
 * IntakeOutAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_INTAKEOUTACTION_H_
#define SRC_ACTIONS_INTAKEOUTACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/IntakeSubsystem.h>

class IntakeOutAction: public CustomAction {
public:
	IntakeOutAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
	}
	~IntakeOutAction() {}

	void start() override {
		intakeSubsystem->setIntakeSpeed(-1);
	};
	void stop() override {};

protected:
	void run() override {};

private:
	IntakeSubsystem *intakeSubsystem;
};

#endif /* SRC_ACTIONS_INTAKEOUTACTION_H_ */
