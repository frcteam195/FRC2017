/*
 * HIDControllerSubsystem.cpp
 *
 *  Created on: Mar 9, 2017
 *      Author: roberthilton
 */

#include <Subsystems/HIDControllerSubsystem.h>

using namespace frc;
using namespace std;

HIDControllerSubsystem::HIDControllerSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	this->subsystemVector = subsystemVector;

	this->robotControllers = robotControllers;
	driveJoystick = robotControllers->getDriveJoystick();
	armJoystick = robotControllers->getArmJoystick();
	buttonBox1 = robotControllers->getButtonBox1();
	buttonBox2 = robotControllers->getButtonBox2();

	runThread = false;
	comingFromAuto = true;

	x = 0, y = 0, absLeft = 0, absRight = 0,normalLeft = 0,normalRight = 0, left = 0, right = 0;

	driveJoystickThreadControlStart = 0;
	driveJoystickThreadControlEnd = 0;
	driveJoystickThreadControlElapsedTimeMS = 0;

	gearButtonCount = 0;

	armJoystickThreadControlStart = 0;
	armJoystickThreadControlEnd = 0;
	armJoystickThreadControlElapsedTimeMS = 0;

	buttonBoxThreadControlStart = 0;
	buttonBoxThreadControlEnd = 0;
	buttonBoxThreadControlElapsedTimeMS = 0;

	climbAction = new ClimbAction(robotControllers, subsystemVector);
	gearPickupAction = new GearPickupAction(robotControllers, subsystemVector);
	gearPlaceAction = new GearPlaceAction(robotControllers, subsystemVector);
	homeGearAction = new HomeGearAction(robotControllers, subsystemVector);
	homeTurretAction = new HomeTurretAction(robotControllers, subsystemVector);
	intakeActuateOutAction = new IntakeActuateOutAction(robotControllers, subsystemVector);
	intakeActuateInAction = new IntakeActuateInAction(robotControllers, subsystemVector);
	intakeInAction = new IntakeInAction(robotControllers, subsystemVector);
	intakeOutAction = new IntakeOutAction(robotControllers, subsystemVector);
	manualTurretControlAction = new ManualTurretControlAction(robotControllers, subsystemVector);
	runBallFeederAction = new RunBallFeederAction(robotControllers, subsystemVector);
	shiftHoldLowAction = new ShiftHoldLowAction(robotControllers, subsystemVector);
	shiftAction = new ShiftAction(robotControllers, subsystemVector);
	shooterAction = new ShooterAction(robotControllers, subsystemVector);
	visionTurretControlAction = new VisionTurretControlAction(robotControllers, subsystemVector);
}

void HIDControllerSubsystem::init() {

}

void HIDControllerSubsystem::start() {
	runThread = true;
	driveJoystickThread = thread(&HIDControllerSubsystem::runDriveJoystickThread, this);
	armJoystickThread = thread(&HIDControllerSubsystem::runArmJoystickThread, this);
	buttonBoxThread = thread(&HIDControllerSubsystem::runButtonBoxThread, this);
}

void HIDControllerSubsystem::subsystemHome() {

}

void HIDControllerSubsystem::stop() {
	runThread = false;
	if(driveJoystickThread.joinable())
		driveJoystickThread.join();
	if(armJoystickThread.joinable())
		armJoystickThread.join();
	if(buttonBoxThread.joinable())
		buttonBoxThread.join();
}

void HIDControllerSubsystem::runDriveJoystickThread() {
	while (!ds->IsEnabled()) {;}
	subsystemHome();

	while(runThread) {
		driveJoystickThreadControlStart = Timer::GetFPGATimestamp();

		if (ds->IsOperatorControl()) {
			if (driveJoystick->GetRisingEdgeButton(DRIVER_PICKUP_GEAR)) {
				gearPickupAction->start();
			} else if (driveJoystick->GetRawButton(DRIVER_PLACE_GEAR)) {
				if(!robotControllers->getGearSwitch()->Get() && gearButtonCount == 0) {
					gearButtonCount++;
					gearPlaceAction->start();
				}
			} else {
				gearButtonCount = 0;
			}

			if (driveJoystick->GetRawButton(DRIVE_SHIFT_HOLDLOW)) {
				shiftHoldLowAction->start(true);
			} else {
				shiftHoldLowAction->start(false);
			}

			if (driveJoystick->GetRisingEdgeButton(DRIVE_SHIFT_LOW)) {
				shiftAction->start(false);
			} else if (driveJoystick->GetRisingEdgeButton(DRIVE_SHIFT_HIGH)) {
				shiftAction->start(true);
			}

			if(!gearPlaceAction->isRunning()) {
				dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()))->changeControlMode(CANSpeedController::kPercentVbus);
				x = driveJoystick->GetRawAxis(DRIVE_X_AXIS);
				y = -driveJoystick->GetRawAxis(DRIVE_Y_AXIS);

				x = abs(x) > 0.1 ? x : 0;
				y = abs(y) > 0.1 ? y : 0;

				if (x != 0)
					x = sgn(x) * ((abs(x) - DRIVE_JOYSTICK_DEADBAND) / (1 - DRIVE_JOYSTICK_DEADBAND));
				if (y != 0)
					y = sgn(y) * ((abs(y) - DRIVE_JOYSTICK_DEADBAND) / (1 - DRIVE_JOYSTICK_DEADBAND));

				left = y + x;
				right = (y - x) * -1;
				absLeft = abs(left);
				absRight = abs(right);

				if(absLeft > absRight && absLeft > 1) {
					normalLeft = left / absLeft;
					normalRight = right / absLeft;
				}
				else if(absRight > absLeft && absRight > 1) {
					normalLeft = left / absRight;
					normalRight = right / absRight;
				}
				else {
					normalLeft = left;
					normalRight = right;
				}

				dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()))->setDriveSpeed(normalLeft, normalRight);
			}
		}

		do {
			driveJoystickThreadControlEnd = Timer::GetFPGATimestamp();
			driveJoystickThreadControlElapsedTimeMS = (int) ((driveJoystickThreadControlEnd - driveJoystickThreadControlStart) * 1000);
			if (driveJoystickThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS)
				this_thread::sleep_for(chrono::milliseconds(MIN_HID_THREAD_LOOP_TIME_MS - driveJoystickThreadControlElapsedTimeMS));
		} while(driveJoystickThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS);
	}
}

void HIDControllerSubsystem::runArmJoystickThread() {
	while (!ds->IsEnabled()) {;}
	subsystemHome();

	while(runThread) {
		armJoystickThreadControlStart = Timer::GetFPGATimestamp();

		if (ds->IsOperatorControl()) {
			if (armJoystick->GetRawButton(SHOOT_BUTTON)) {
				runBallFeederAction->start();
				comingFromAuto = false;
			} else if (armJoystick->GetRawButton(AGITATOR_REVERSE)) {
				runBallFeederAction->start(true);
				comingFromAuto = false;
			} else if(armJoystick->GetRawButton(SHOOTER_OFF_BUTTON)) {
				runBallFeederAction->stop();
				shooterAction->stop();
				comingFromAuto = false;
			}else {
				if (!comingFromAuto)
					runBallFeederAction->stop();
			}

			if (armJoystick->GetRisingEdgeButton(SPEED_UP)) {
				shooterAction->bumpShooterSpeed(true);
			} else if (armJoystick->GetRisingEdgeButton(SPEED_DOWN)) {
				shooterAction->bumpShooterSpeed(false);
			}

			if (armJoystick->GetRisingEdgeButton(HOOD_UP)) {
				shooterAction->bumpHood(true);
			} else if (armJoystick->GetRisingEdgeButton(HOOD_DOWN)) {
				shooterAction->bumpHood(false);
			}

			if (armJoystick->GetRawButton(MANUAL_OVERRIDE_BUTTON)) {
				visionTurretControlAction->stop();
				manualTurretControlAction->start();
			} else if (armJoystick->GetRawButton(VISION_HOLD_ON)) {
				visionTurretControlAction->holdStart();
			} else if (armJoystick->GetRisingEdgeButton(VISION_SET_ON)) {
				visionTurretControlAction->start();
			} else if (armJoystick->GetRisingEdgeButton(VISION_SET_OFF)) {
				visionTurretControlAction->stop();
			} else if (buttonBox2->GetRisingEdgeButton(REHOME_TURRET)) {
				//homeTurretAction->start();
			}

		}

		do {
			armJoystickThreadControlEnd = Timer::GetFPGATimestamp();
			armJoystickThreadControlElapsedTimeMS = (int) ((armJoystickThreadControlEnd - armJoystickThreadControlStart) * 1000);
			if (armJoystickThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS)
				this_thread::sleep_for(chrono::milliseconds(MIN_HID_THREAD_LOOP_TIME_MS - armJoystickThreadControlElapsedTimeMS));
		} while(armJoystickThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS);
	}
}

void HIDControllerSubsystem::runButtonBoxThread() {
	while (!ds->IsEnabled()) {;}
	subsystemHome();

	while(runThread) {
		buttonBoxThreadControlStart = Timer::GetFPGATimestamp();

		if (ds->IsOperatorControl()) {
			if (buttonBox1->GetRisingEdgeButton(INTAKE_ACTUATE_OUT_BUTTON) || driveJoystick->GetRawButton(DRIVE_INTAKE_OUT_BUTTON)) {
				intakeActuateOutAction->start();
			} else if (buttonBox1->GetRisingEdgeButton(INTAKE_ACTUATE_IN_BUTTON) || driveJoystick->GetRawButton(DRIVE_INTAKE_IN_BUTTON)) {
				intakeActuateInAction->start();
			}

			if (buttonBox1->GetRawButton(INTAKE_OUT_BUTTON)) {
				intakeOutAction->start();
			} else if (buttonBox1->GetRawButton(INTAKE_IN_BUTTON) || armJoystick->GetRawButton(ARM_INTAKE_IN_BUTTON) || driveJoystick->GetRawAxis(2) < -0.5) {
				intakeInAction->start();
			} else if (buttonBox2->GetRawButton(CLIMB_BUTTON)) {
				climbAction->start();
			} else {
				intakeInAction->stop();
			}

			if (buttonBox1->GetRisingEdgeButton(PICKUP_GEAR)) {
				gearPickupAction->start();
			} else if (buttonBox1->GetRawButton(PLACE_GEAR)) {
				if(!robotControllers->getGearSwitch()->Get()) {
					gearPlaceAction->start();
				}
			}

			if (buttonBox2->GetRisingEdgeButton(REHOME_GEAR)) {
				homeGearAction->start();
			}

			if (buttonBox1->GetRisingEdgeButton(SHOOT_FROM_HOPPER)) {
				visionTurretControlAction->stop();
				shooterAction->start(ShotSelection::hopper);
				visionTurretControlAction->start();
			} else if (buttonBox1->GetRisingEdgeButton(SHOOT_FROM_CENTER_GEAR)) {
				visionTurretControlAction->stop();
				shooterAction->start(ShotSelection::gearCenter);
				visionTurretControlAction->start();
			} else if (buttonBox1->GetRisingEdgeButton(SHOOT_FROM_SIDE_GEAR)) {
				visionTurretControlAction->stop();
				shooterAction->start(ShotSelection::gearFar);
				visionTurretControlAction->start();
			} else if (buttonBox1->GetRisingEdgeButton(SHOOT_FROM_LOADING_ZONE)) {
				visionTurretControlAction->stop();
				shooterAction->start(ShotSelection::loadingZone);
				visionTurretControlAction->start();
			} else if (buttonBox2->GetRisingEdgeButton(SHOOT_REMOVE_BALLS)) {
				visionTurretControlAction->stop();
				shooterAction->start(ShotSelection::removeBalls);
				visionTurretControlAction->start();
			}

		}

		do {
			buttonBoxThreadControlEnd = Timer::GetFPGATimestamp();
			buttonBoxThreadControlElapsedTimeMS = (int) ((buttonBoxThreadControlEnd - buttonBoxThreadControlStart) * 1000);
			if (buttonBoxThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS)
				this_thread::sleep_for(chrono::milliseconds(MIN_HID_THREAD_LOOP_TIME_MS - buttonBoxThreadControlElapsedTimeMS));
		} while(buttonBoxThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS);
	}
}
