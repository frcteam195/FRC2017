/*
 * RunBallFeederAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_RUNBALLFEEDERACTION_H_
#define SRC_ACTIONS_RUNBALLFEEDERACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/BallFeederSubsystem.h>

class RunBallFeederAction: public CustomAction {
public:
	RunBallFeederAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		ballFeederSubsystem = dynamic_cast<BallFeederSubsystem*>(subsystemVector->at(robotControllers->getFeederSubsystemIndex()));
	}
	~RunBallFeederAction() {}

	void start(bool reverse) {
		run(true, reverse);
	};

	void start() override {
		start(false);
	};

	void stop() override {
		run(false, false);
	};

protected:
	void run() override {};

private:
	BallFeederSubsystem *ballFeederSubsystem;

	void run(bool on, bool reverse) {
		if (reverse && on)
			ballFeederSubsystem->setFeeder(1,1);
		else if (on) {
			//ballFeederSubsystem->setFeeder(-0.75,-1);
			//ballFeederSubsystem->setFeeder(-0.75,-0.72);
			ballFeederSubsystem->setFeeder(-1, -0.75);
		}
		else
			ballFeederSubsystem->setFeeder(0,0);
	};
};

#endif /* SRC_ACTIONS_RUNBALLFEEDERACTION_H_ */
