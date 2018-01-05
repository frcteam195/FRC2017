/*
 * IntakeActuateOutAction.h
 *
 *  Created on: Mar 10, 2017
 *      Author: chris
 */

#ifndef SRC_ACTIONS_INTAKEACTUATEOUTACTION_H_
#define SRC_ACTIONS_INTAKEACTUATEOUTACTION_H_

#include <Utilities/CustomAction.h>
#include "Subsystems/IntakeSubsystem.h"

class IntakeActuateOutAction: public CustomAction {
public:
	IntakeActuateOutAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector) {
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
	}
	~IntakeActuateOutAction() {};

	void start() override {
		intakeSubsystem->setIntakePosition(true);
	};
	void stop() override {};

protected:
	void run() override {};

private:
	IntakeSubsystem *intakeSubsystem;
};

#endif /* SRC_ACTIONS_INTAKEACTUATEOUTACTION_H_ */
