/*
 * HomeGearAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_HOMEGEARACTION_H_
#define SRC_ACTIONS_HOMEGEARACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/GearSubsystem.h>

class HomeGearAction: public CustomAction {
public:
	HomeGearAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		gearSubsystem = dynamic_cast<GearSubsystem*>(subsystemVector->at(robotControllers->getGearSubsystemIndex()));
	}
	~HomeGearAction() {}

	void start() override {
		gearSubsystem->subsystemHome();
	};
	void stop() override {};

protected:
	void run() override {};

private:
	GearSubsystem *gearSubsystem;
};

#endif /* SRC_ACTIONS_HOMEGEARACTION_H_ */
