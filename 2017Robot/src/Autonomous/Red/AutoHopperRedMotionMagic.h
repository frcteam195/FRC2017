/*
 * AutoHopperRedMotionMagic.h
 *
 *  Created on: Apr 28, 2017
 *      Author: roberthilton
 */

#ifndef SRC_AUTONOMOUS_RED_AUTOHOPPERREDMOTIONMAGIC_H_
#define SRC_AUTONOMOUS_RED_AUTOHOPPERREDMOTIONMAGIC_H_


#include <Utilities/CustomAction.h>
#include <Actions/AgitateAction.h>
#include <Actions/VisionTurretControlAction.h>
#include <Subsystems/DriveBaseSubsystem.h>
#include <Subsystems/ShooterSubsystem.h>
#include <Subsystems/TurretSubsystem.h>
#include <Subsystems/BallFeederSubsystem.h>
#include <Subsystems/IntakeSubsystem.h>

#define AUTO_HOPPER_RED_TARGETING_TIMEOUT_MS 500

class AutoHopperRedMotionMagic: public CustomAction {
public:
	AutoHopperRedMotionMagic(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		ds = &DriverStation::GetInstance();
		driveBaseSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()));
		shooterSubsystem = dynamic_cast<ShooterSubsystem*>(subsystemVector->at(robotControllers->getShooterSubsystemIndex()));
		turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));
		ballFeederSubsystem = dynamic_cast<BallFeederSubsystem*>(subsystemVector->at(robotControllers->getFeederSubsystemIndex()));
		intakeSubsystem = dynamic_cast<IntakeSubsystem*>(subsystemVector->at(robotControllers->getIntakeSubsystemIndex()));
		agitateAction = new AgitateAction(robotControllers, subsystemVector);
		visionTurretControlAction = new VisionTurretControlAction(robotControllers, subsystemVector);
	}
	~AutoHopperRedMotionMagic() {}

	void start() override {
		driveBaseSubsystem->changeTalonControlMode(CANTalon::kMotionMagicMode);
		driveBaseSubsystem->setDrivePID(0.3, 0, 0, 0.5, 0);
		driveBaseSubsystem->setMotionMagicVelocityAccel(2000, 9000);
		this_thread::sleep_for(chrono::milliseconds(150));
		driveBaseSubsystem->setGear(false);
		intakeSubsystem->setIntakePosition(true);
		turretSubsystem->selectShot(ShotSelection::hopper);
		//driveBaseSubsystem->changeControlMode(CANSpeedController::kMotionProfile);
		//driveBaseSubsystem->processMotionProfile(getVectorFromLeftProfile(), getVectorFromRightProfile());


		double avgPosTmp = 0;

		driveBaseSubsystem->setDriveSpeed(33.5, -33.5);

		do {
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (driveBaseSubsystem->isPositionWithinRange(0.5) && ds->IsAutonomous());

		cout << "turning" << endl;

		driveBaseSubsystem->changeControlMode(CANSpeedController::kPercentVbus);

		driveBaseSubsystem->setDriveSpeed(0.75, 0.75);

		shooterSubsystem->selectShot(ShotSelection::hopper);

		avgPosTmp = 0;

		do {
			avgPosTmp = abs(robotControllers->getNavX()->GetYaw());
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (avgPosTmp < 82 && ds->IsAutonomous());
		driveBaseSubsystem->setDriveSpeed(0, 0);

		driveBaseSubsystem->changeTalonControlMode(CANTalon::kMotionMagicMode);

		driveBaseSubsystem->setDriveSpeed(driveBaseSubsystem->getLeftDrivePosition() + 17, driveBaseSubsystem->getRightDrivePosition() - 17);

		do {
			this_thread::sleep_for(chrono::milliseconds(10));
		} while (driveBaseSubsystem->isPositionWithinRange(1) && ds->IsAutonomous());

		visionTurretControlAction->start();

		timeoutStart = Timer::GetFPGATimestamp();
		while (!visionTurretControlAction->isOnTarget() && timeoutElapsedTimeMS < AUTO_HOPPER_RED_TARGETING_TIMEOUT_MS) {
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}

		//delete visionTurretControlAction;
		//visionTurretControlAction = nullptr;

		ballFeederSubsystem->setFeederOn();


		this_thread::sleep_for(chrono::milliseconds(2000));
		agitateAction->start();
		timeoutStart = Timer::GetFPGATimestamp();
		while (ds->IsAutonomous()&&ds->IsEnabled()) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}
		agitateAction->stop();
		intakeSubsystem->setIntakePosition(true);
		visionTurretControlAction->stop();
		//ballFeederSubsystem->setFeederOff();

		driveBaseSubsystem->changeControlMode(CANSpeedController::kPercentVbus);
		driveBaseSubsystem->setDriveSpeed(0, 0);
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

	AgitateAction *agitateAction;
	VisionTurretControlAction *visionTurretControlAction;
};


#endif /* SRC_AUTONOMOUS_RED_AUTOHOPPERREDMOTIONMAGIC_H_ */
