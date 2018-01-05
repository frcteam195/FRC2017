/*
 * GearOscillatorAction.h
 *
 *  Created on: Mar 23, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_GEAROSCILLATORACTION_H_
#define SRC_ACTIONS_GEAROSCILLATORACTION_H_

#include <Utilities/CustomAction.h>
#include "Subsystems/GearSubsystem.h"

#define GEAR_OSCILLATOR_TIME_FILTER 25

class GearOscillatorAction: public CustomAction {
public:
	GearOscillatorAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector): CustomAction(robotControllers, subsystemVector) {
		gearSubsystem =  dynamic_cast<GearSubsystem*>(subsystemVector->at(robotControllers->getGearSubsystemIndex()));
		gearOscillatorCounter = 0;
	}

	~GearOscillatorAction() {}

	void start() override {
		if (gearOscillatorCounter++ >= GEAR_OSCILLATOR_TIME_FILTER) {
			//if (gearSubsystem->getGearActuatorSetpoint() <= POSITION_RELEASE) {
				gearSubsystem->setGearActuatorPos(POSITION_ZERO);
			//} else if (gearSubsystem->getGearActuatorSetpoint() >= POSITION_ZERO) {
				gearSubsystem->setGearActuatorPos(-0.04);
			//}
			gearOscillatorCounter = 0;
		}
	};

	void stop() override {};

protected:
	void run() override {};

private:
	GearSubsystem *gearSubsystem;
	int gearOscillatorCounter;
};



#endif /* SRC_ACTIONS_GEAROSCILLATORACTION_H_ */
