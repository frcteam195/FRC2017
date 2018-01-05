/*
 * AutoBlueBoilerSideGearHopper.h
 *
 *  Created on: Mar 14, 2017
 *      Author: roberthilton
 */

#ifndef SRC_AUTONOMOUS_BLUE_AUTOBLUEBOILERSIDEGEARHOPPER_H_
#define SRC_AUTONOMOUS_BLUE_AUTOBLUEBOILERSIDEGEARHOPPER_H_

#include <Utilities/CustomAction.h>
#include <Actions/VisionTurretControlAction.h>
#include <Actions/GearPlaceAction.h>
#include <Subsystems/DriveBaseSubsystem.h>
#include <Subsystems/ShooterSubsystem.h>
#include <Subsystems/TurretSubsystem.h>
#include <Subsystems/BallFeederSubsystem.h>
#include <Subsystems/IntakeSubsystem.h>

#define AUTO_HOPPER_GEAR_BLUE_TARGETING_TIMEOUT_MS 750
#define AUTO_SIDE_GEAR_PLACING_TIMEOUT_MS 1500

class AutoBlueBoilerSideGearHopper: public CustomAction {
public:
	AutoBlueBoilerSideGearHopper(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
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
	~AutoBlueBoilerSideGearHopper() {}

	void start() override {
		this_thread::sleep_for(chrono::milliseconds(250));
		driveBaseSubsystem->setGear(false);

		turretSubsystem->selectShot(ShotSelection::hopper);
		shooterSubsystem->selectShot(ShotSelection::hopper);
		//shooterSubsystem->setSpeed(-3400);
		//shooterSubsystem->setHoodPos(0.362);

		double avgPosTmp = 0;

		driveBaseSubsystem->setDriveSpeed(-1, 1);

		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 22 && ds->IsAutonomous());

		intakeSubsystem->setIntakePosition(true);

		driveBaseSubsystem->setDriveSpeed(0.75, 0.75);

		avgPosTmp = 0;

		do {
			avgPosTmp = abs(robotControllers->getNavX()->GetYaw());
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 47 && ds->IsAutonomous());

		driveBaseSubsystem->setDriveSpeed(0, 0);
		this_thread::sleep_for(chrono::milliseconds(100));
		driveBaseSubsystem->setPosition(0);
		this_thread::sleep_for(chrono::milliseconds(40));

		driveBaseSubsystem->setDriveSpeed(-1, 1);

		avgPosTmp = 0;

		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 24 && ds->IsAutonomous());

		driveBaseSubsystem->setDriveSpeed(0, 0);

		timeoutStart = Timer::GetFPGATimestamp();
		while (robotControllers->getGearSwitch()->Get() && timeoutElapsedTimeMS < AUTO_SIDE_GEAR_PLACING_TIMEOUT_MS) {
			//driveBaseSubsystem->setDriveSpeed(-0.17, 0.17);
			driveBaseSubsystem->setDriveSpeed(-0.25, 0.25);
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}

		driveBaseSubsystem->setDriveSpeed(0, 0);

		gearPlaceAction->start();

		timeoutStart = Timer::GetFPGATimestamp();
		while (gearPlaceAction->isRunning() && timeoutElapsedTimeMS < AUTO_SIDE_GEAR_PLACING_TIMEOUT_MS) {
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}

		driveBaseSubsystem->setDriveSpeed(0, 0);
		this_thread::sleep_for(chrono::milliseconds(100));
		driveBaseSubsystem->setPosition(0);
		this_thread::sleep_for(chrono::milliseconds(40));
		driveBaseSubsystem->setPosition(0);
		this_thread::sleep_for(chrono::milliseconds(40));

		driveBaseSubsystem->setDriveSpeed(1, -1);

		avgPosTmp = 0;

		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 10.5 && ds->IsAutonomous());

		driveBaseSubsystem->setDriveSpeed(0, 0);


		driveBaseSubsystem->setDriveSpeed(0.75, 0.75);

		avgPosTmp = 0;

		do {
			avgPosTmp = abs(robotControllers->getNavX()->GetYaw());
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 82.5 && ds->IsAutonomous());

		driveBaseSubsystem->setDriveSpeed(0, 0);
		this_thread::sleep_for(chrono::milliseconds(100));
		driveBaseSubsystem->setPosition(0);
		this_thread::sleep_for(chrono::milliseconds(40));

		driveBaseSubsystem->setDriveSpeed(1, -1);

		avgPosTmp = 0;

		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 25 && ds->IsAutonomous());

		driveBaseSubsystem->setDriveSpeed(0, 0);
		this_thread::sleep_for(chrono::milliseconds(200));

		visionTurretControlAction->start();
		timeoutStart = Timer::GetFPGATimestamp();
		int ontargetFilter = 0;
		while (ontargetFilter < 4 && timeoutElapsedTimeMS < AUTO_HOPPER_GEAR_BLUE_TARGETING_TIMEOUT_MS) {
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
			if (visionTurretControlAction->isOnTarget())
				ontargetFilter++;
		}
		//
		//delete visionTurretControlAction;
		//visionTurretControlAction = nullptr;

		ballFeederSubsystem->setFeederOn();

		this_thread::sleep_for(chrono::milliseconds(1000));

		timeoutStart = Timer::GetFPGATimestamp();
		while (ds->IsAutonomous()&&ds->IsEnabled()) {

			timeoutEnd = Timer::GetFPGATimestamp();
			if ((timeoutEnd - timeoutStart) > 2.5) {
				timeoutStart = Timer::GetFPGATimestamp();
				intakeSubsystem->setIntakePosition(false);
				this_thread::sleep_for(chrono::milliseconds(100));
				intakeSubsystem->setIntakeSpeed(1);
				this_thread::sleep_for(chrono::milliseconds(250));
				intakeSubsystem->setIntakeSpeed(0);
				intakeSubsystem->setIntakePosition(true);
			}
		}
		intakeSubsystem->setIntakePosition(true);
		//ballFeederSubsystem->setFeederOff();
		visionTurretControlAction->stop();
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

#endif /* SRC_AUTONOMOUS_BLUE_AUTOBLUEBOILERSIDEGEARHOPPER_H_ */
