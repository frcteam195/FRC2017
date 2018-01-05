/*
 * AgitateAction.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chris
 */

#ifndef SRC_ACTIONS_AGITATEACTION_H_
#define SRC_ACTIONS_AGITATEACTION_H_

#include "Utilities/CustomAction.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/DriveBaseSubsystem.h"

class AgitateAction: public CustomAction {
public:
	AgitateAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		ds = &DriverStation::GetInstance();
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
		driveBaseSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()));
		timeoutShort = 0;
	}
	~AgitateAction() {}

	void start() override {
		if (_actionMutex.try_lock()) {
			running = true;
			runAction = thread(&AgitateAction::run, this);
		}
	};
	void stop() override {};

protected:
	void run() override {
		bool prevGear = driveBaseSubsystem->isHighGear();

		driveBaseSubsystem->setGear(!driveBaseSubsystem->isHighGear());
		this_thread::sleep_for(chrono::milliseconds(450));
		driveBaseSubsystem->setGear(!driveBaseSubsystem->isHighGear());

		intakeSubsystem->setIntakePosition(false);
		this_thread::sleep_for(chrono::milliseconds(450));
		intakeSubsystem->setIntakePosition(true);

		timeoutStart = Timer::GetFPGATimestamp();
		timeoutShort = Timer::GetFPGATimestamp();

		while(robotControllers->getDriveJoystick()->GetRawButton(DRIVE_AGITATE) || ds->IsAutonomous()) {
			timeoutEnd = Timer::GetFPGATimestamp();

			if((timeoutEnd - timeoutShort) >= 1) {
				timeoutShort = Timer::GetFPGATimestamp();
				driveBaseSubsystem->setGear(!driveBaseSubsystem->isHighGear());
				this_thread::sleep_for(chrono::milliseconds(450));
				driveBaseSubsystem->setGear(!driveBaseSubsystem->isHighGear());
			}
			if((timeoutEnd - timeoutStart) >= 5.5) {
				timeoutStart = Timer::GetFPGATimestamp();
				intakeSubsystem->setIntakePosition(false);
				this_thread::sleep_for(chrono::milliseconds(450));
				intakeSubsystem->setIntakePosition(true);
			}
		}
		intakeSubsystem->setIntakePosition(true);
		driveBaseSubsystem->setGear(prevGear);

		running = false;
		runAction.detach();
		_actionMutex.unlock();
	};

private:
	IntakeSubsystem *intakeSubsystem;
	DriverStation *ds;
	DriveBaseSubsystem *driveBaseSubsystem;
	double timeoutShort;
};

#endif /* SRC_ACTIONS_AGITATEACTION_H_ */
