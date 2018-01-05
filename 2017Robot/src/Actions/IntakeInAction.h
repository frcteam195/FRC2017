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
#include <Subsystems/BallFeederSubsystem.h>
#include <Subsystems/ShooterSubsystem.h>


class IntakeInAction: public CustomAction {
public:
	IntakeInAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
		feederSubsystem = dynamic_cast<BallFeederSubsystem*>(subsystemVector->at(robotControllers->getFeederSubsystemIndex()));
		shooterSubsystem = dynamic_cast<ShooterSubsystem*>(subsystemVector->at(robotControllers->getShooterSubsystemIndex()));
		runCount = 0;
	}
	~IntakeInAction() {}

	void start() override {
		if (!intakeSubsystem->isIntakeLocked()) {
			intakeSubsystem->setIntakeSpeed(1);
			if(shooterSubsystem->getSetpoint() == 0) {
				feederSubsystem->setFeeder(-1, 0);
				runCount = 1;
			}
		} else {
			feederSubsystem->setFeeder(0, 0);
		}
	};

	void stop() override {
		intakeSubsystem->setIntakeSpeed(0);
		if(runCount != 0) {
			feederSubsystem->setFeeder(0, 0);
			runCount = 0;
		}
	};

protected:
	void run() override {};

private:
	IntakeSubsystem *intakeSubsystem;
	BallFeederSubsystem *feederSubsystem;
	ShooterSubsystem *shooterSubsystem;
	int runCount;
};

#endif /* SRC_ACTIONS_INTAKEINACTION_H_ */
