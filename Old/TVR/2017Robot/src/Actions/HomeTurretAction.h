/*
 * TurretHomeAction.h
 *
 *  Created on: Mar 14, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_HOMETURRETACTION_H_
#define SRC_ACTIONS_HOMETURRETACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/TurretSubsystem.h>

class HomeTurretAction: public CustomAction {
public:
	HomeTurretAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));
	}
	~HomeTurretAction() {}

	void start() override {
		turretSubsystem->subsystemHome();
	};
	void stop() override {};

protected:
	void run() override {};

private:
	TurretSubsystem *turretSubsystem;
};

#endif /* SRC_ACTIONS_HOMETURRETACTION_H_ */
