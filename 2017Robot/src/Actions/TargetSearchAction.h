/*
 * TargetSearch.h
 *
 *  Created on: Apr 21, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_TARGETSEARCHACTION_H_
#define SRC_ACTIONS_TARGETSEARCHACTION_H_


#include <Utilities/CustomAction.h>
#include <Subsystems/TurretSubsystem.h>

class TargetSearchAction: public CustomAction {
public:
	TargetSearchAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));
	}
	~TargetSearchAction() {}

	void start() override {
		turretSubsystem->configVelocityAccel(1200, 2800);
		turretSubsystem->setTurretPos(1.7);

		timeoutStart = Timer::GetFPGATimestamp();

		while (abs(turretSubsystem->getTurretPos() - 1.5) > 0.1 && timeoutElapsedTimeMS < 2000) {
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}
		turretSubsystem->configVelocityAccel(200, 800);
		//this_thread::sleep_for(chrono::milliseconds(300));
		turretSubsystem->setTurretPos(-4.7);
	};
	void stop() override {};

protected:
	void run() override {};

private:
	TurretSubsystem *turretSubsystem;
};


#endif /* SRC_ACTIONS_TARGETSEARCHACTION_H_ */
