/*
 * ShooterWheelAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_SHOOTERACTION_H_
#define SRC_ACTIONS_SHOOTERACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/ShooterSubsystem.h>
#include <Subsystems/TurretSubsystem.h>

#define TURRET_ACTION_TIMEOUT_MS 1000
#define TURRET_DEVIATION 0.1

class ShooterAction: public CustomAction {
public:
	ShooterAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		shooterSubsystem = dynamic_cast<ShooterSubsystem*>(subsystemVector->at(robotControllers->getShooterSubsystemIndex()));
		turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));
	}
	~ShooterAction() {}

	void bumpShooterSpeed(bool increase) {
		shooterSubsystem->bumpShooterSpeed(increase);
	}

	void bumpHood(bool increase) {
		shooterSubsystem->bumpHood(increase);
	}

	void start(ShotSelection shot) {
		shooterSubsystem->selectShot(shot);
		turretSubsystem->selectShot(shot);
		timeoutStart = Timer::GetFPGATimestamp();
		while(abs(turretSubsystem->getTurretPos() - turretSubsystem->getTurretSetpoint()) > TURRET_DEVIATION && timeoutElapsedTimeMS < TURRET_ACTION_TIMEOUT_MS) {
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}
	};

	void start() override {
		shooterSubsystem->selectShot(ShotSelection::hopper);
	};

	void stop() override {
		shooterSubsystem->setSpeed(0);
	};

protected:
	void run() override {};

private:
	ShooterSubsystem *shooterSubsystem;
	TurretSubsystem *turretSubsystem;
};

#endif /* SRC_ACTIONS_SHOOTERACTION_H_ */
