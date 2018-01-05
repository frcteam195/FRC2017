/*
 * AutoRedBoilerSideGear.h
 *
 *  Created on: Mar 14, 2017
 *      Author: roberthilton
 */

#ifndef SRC_AUTONOMOUS_AUTOREDBOILERSIDEGEAR_H_
#define SRC_AUTONOMOUS_AUTOREDBOILERSIDEGEAR_H_

#include <Utilities/CustomAction.h>
#include <Actions/VisionTurretControlAction.h>
#include <Actions/GearPlaceAction.h>
#include <Subsystems/DriveBaseSubsystem.h>
#include <Subsystems/ShooterSubsystem.h>
#include <Subsystems/TurretSubsystem.h>
#include <Subsystems/BallFeederSubsystem.h>
#include <Subsystems/IntakeSubsystem.h>

#define AUTO_HOPPER_RED_TARGETING_TIMEOUT_MS 500
#define AUTO_SIDE_GEAR_PLACING_TIMEOUT_MS 1500

class AutoRedBoilerSideGear: public CustomAction {
public:
	AutoRedBoilerSideGear(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
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
	~AutoRedBoilerSideGear() {}

	void start() override {
		this_thread::sleep_for(chrono::milliseconds(250));
		driveBaseSubsystem->setGear(false);

		turretSubsystem->selectShot(ShotSelection::gearFar);
		//shooterSubsystem->selectShot(ShotSelection::gearFar);
		shooterSubsystem->setSpeed(-3400);
		shooterSubsystem->setHoodPos(0.362);

		double avgPosTmp = 0;

		driveBaseSubsystem->setDriveSpeed(-1, 1);

		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 26.5 && ds->IsAutonomous());

		intakeSubsystem->setIntakePosition(true);

		driveBaseSubsystem->setDriveSpeed(-0.75, -0.75);

		avgPosTmp = 0;

		do {
			avgPosTmp = abs(robotControllers->getNavX()->GetYaw());
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 47 && ds->IsAutonomous());
		driveBaseSubsystem->setDriveSpeed(0, 0);
		this_thread::sleep_for(chrono::milliseconds(100));
		driveBaseSubsystem->setPosition(0);
		this_thread::sleep_for(chrono::milliseconds(30));

		driveBaseSubsystem->setDriveSpeed(-1, 1);

		avgPosTmp = 0;

		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			cout << "average pos: " << avgPosTmp << endl;
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 24 && ds->IsAutonomous());

		driveBaseSubsystem->setDriveSpeed(0, 0);

		timeoutStart = Timer::GetFPGATimestamp();
		while (robotControllers->getGearSwitch()->Get() && timeoutElapsedTimeMS < AUTO_SIDE_GEAR_PLACING_TIMEOUT_MS) {
			driveBaseSubsystem->setDriveSpeed(-0.17, 0.17);
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}

		driveBaseSubsystem->setDriveSpeed(0, 0);

		gearPlaceAction->start();

		visionTurretControlAction->start();
		timeoutStart = Timer::GetFPGATimestamp();
		while (!visionTurretControlAction->isOnTarget() && timeoutElapsedTimeMS < AUTO_HOPPER_RED_TARGETING_TIMEOUT_MS) {
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}
		visionTurretControlAction->stop();
		//delete visionTurretControlAction;
		//visionTurretControlAction = nullptr;

		this_thread::sleep_for(chrono::milliseconds(2000));

		ballFeederSubsystem->setFeederOn();

		this_thread::sleep_for(chrono::milliseconds(1000));

		timeoutStart = Timer::GetFPGATimestamp();
		while (ds->IsAutonomous()&&ds->IsEnabled()) {

			timeoutEnd = Timer::GetFPGATimestamp();
			if ((timeoutEnd - timeoutStart) > 3) {
				timeoutStart = Timer::GetFPGATimestamp();
				intakeSubsystem->setIntakePosition(false);
				intakeSubsystem->setIntakeSpeed(1);
				this_thread::sleep_for(chrono::milliseconds(350));
				intakeSubsystem->setIntakeSpeed(0);
				intakeSubsystem->setIntakePosition(true);
			}
		}
		intakeSubsystem->setIntakePosition(true);
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

#endif /* SRC_AUTONOMOUS_AUTOREDBOILERSIDEGEAR_H_ */
