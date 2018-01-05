/*
 * AutoRedCenterGear.h
 *
 *  Created on: Apr 22, 2017
 *      Author: roberthilton
 */

#ifndef SRC_AUTONOMOUS_RED_AUTOREDCENTERGEAR_H_
#define SRC_AUTONOMOUS_RED_AUTOREDCENTERGEAR_H_

#include <Utilities/CustomAction.h>
#include <Actions/VisionTurretControlAction.h>
#include <Actions/GearPlaceAction.h>
#include <Subsystems/DriveBaseSubsystem.h>
#include <Subsystems/ShooterSubsystem.h>
#include <Subsystems/TurretSubsystem.h>
#include <Subsystems/BallFeederSubsystem.h>
#include <Subsystems/IntakeSubsystem.h>

#define AUTO_HOPPER_BLUE_TARGETING_TIMEOUT_MS 500

class AutoRedCenterGear: public CustomAction {
public:
	AutoRedCenterGear(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		ds = &DriverStation::GetInstance();
		driveBaseSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()));
		shooterSubsystem = dynamic_cast<ShooterSubsystem*>(subsystemVector->at(robotControllers->getShooterSubsystemIndex()));
		turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));
		ballFeederSubsystem = dynamic_cast<BallFeederSubsystem*>(subsystemVector->at(robotControllers->getFeederSubsystemIndex()));
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
		gearPlaceAction = new GearPlaceAction(robotControllers, subsystemVector);
		visionTurretControlAction = new VisionTurretControlAction(robotControllers, subsystemVector);
	}
	~AutoRedCenterGear() {}

	void start() override {
		this_thread::sleep_for(chrono::milliseconds(250));
		driveBaseSubsystem->setGear(false);



		double avgPosTmp = 0;

		//double startPos = driveBaseSubsystem->getAveragePosition();
		driveBaseSubsystem->setDriveSpeed(-.5, .5);

		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 10 && ds->IsAutonomous());

		intakeSubsystem->setIntakePosition(true);

		turretSubsystem->selectShot(ShotSelection::gearCenter);
		shooterSubsystem->selectShot(ShotSelection::gearCenter);
		shooterSubsystem->setSpeed(-3600);

		//startPos = driveBaseSubsystem->getAveragePosition();

		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			cout << "average pos: " << avgPosTmp << endl;
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 25 && ds->IsAutonomous());

		//driveBaseSubsystem->setDriveSpeed(0, 0);

		timeoutStart = Timer::GetFPGATimestamp();
		while (robotControllers->getGearSwitch()->Get() && timeoutElapsedTimeMS < 3000) {
			driveBaseSubsystem->setDriveSpeed(-0.17, 0.17);
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}

		driveBaseSubsystem->setDriveSpeed(0, 0);

		gearPlaceAction->start();

		visionTurretControlAction->start();
		timeoutStart = Timer::GetFPGATimestamp();
		while (!visionTurretControlAction->isOnTarget() && timeoutElapsedTimeMS < AUTO_HOPPER_BLUE_TARGETING_TIMEOUT_MS) {
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}

		this_thread::sleep_for(chrono::milliseconds(1000));

		ballFeederSubsystem->setFeederOn();

		this_thread::sleep_for(chrono::milliseconds(1000));

		timeoutStart = Timer::GetFPGATimestamp();

		while (ds->IsAutonomous()&&ds->IsEnabled()) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		visionTurretControlAction->stop();
		intakeSubsystem->setIntakePosition(true);
		shooterSubsystem->setSpeed(0);
		turretSubsystem->setTurretPos(0);
		ballFeederSubsystem->setFeederOff();
	};

	void stop() override {

	};

protected:
	void run() override {};

private:
	DriverStation *ds;

	DriveBaseSubsystem *driveBaseSubsystem;
	ShooterSubsystem *shooterSubsystem;
	TurretSubsystem *turretSubsystem;
	BallFeederSubsystem *ballFeederSubsystem;
	IntakeSubsystem *intakeSubsystem;

	GearPlaceAction *gearPlaceAction;
	VisionTurretControlAction *visionTurretControlAction;

};


#endif /* SRC_AUTONOMOUS_RED_AUTOREDCENTERGEAR_H_ */
