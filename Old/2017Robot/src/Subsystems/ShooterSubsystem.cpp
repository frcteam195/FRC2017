#include "Subsystems/ShooterSubsystem.h"
#include <iostream>

using namespace std;
using namespace frc;

ShooterSubsystem::ShooterSubsystem(Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector)
: TuneablePID("Shooter", robotControllers->getShooterWheel(), UDP_PORT_NUMBER, true, true) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	buttonBox1 = robotControllers->getButtonBox1();
	armJoystick = robotControllers->getArmJoystick();

	shooterWheel = robotControllers->getShooterWheel();
	hoodMotor = robotControllers->getHoodMotor();

	shooterVelocity = 0;

	runThread = false;

	shooterThreadControlStart = 0;
	shooterThreadControlEnd = 0;
	shooterThreadControlElapsedTimeMS = 0;
}

void ShooterSubsystem::init() {
	shooterWheel->SetStatusFrameRateMs(CANTalon::StatusFrameRateQuadEncoder, 1);
	shooterWheel->SetStatusFrameRateMs(CANTalon::StatusFrameRateFeedback, 1);
	shooterWheel->SetVelocityMeasurementPeriod(CANTalon::Period_20Ms);
	shooterWheel->SetVelocityMeasurementWindow(32);
	shooterWheel->SetControlMode(CANSpeedController::ControlMode::kSpeed);
	shooterWheel->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	shooterWheel->SetClosedLoopOutputDirection(true);
	//shooterWheel->SetCurrentLimit(100);
	shooterWheel->SetPID(0.25, 0, 5, 0.0268);
	shooterVelocity = -3625;

	hoodMotor->SetMotionMagicCruiseVelocity(1200);
	hoodMotor->SetMotionMagicAcceleration(9000);
	hoodMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	hoodMotor->SetSensorDirection(false);
	hoodMotor->SetPID(20, 0, 200, 1.1352539063);
	hoodMotor->SetTalonControlMode(CANTalon::kMotionMagicMode);
	hoodMotor->SetEncPosition(0);
	hoodMotor->ConfigSoftPositionLimits(HOOD_UPPER_SOFT_STOP, HOOD_LOWER_SOFT_STOP);
	hoodMotor->Set(0);
}

void ShooterSubsystem::start() {
	runThread = true;
	shooterThread = thread(&ShooterSubsystem::assignOutput, this);
}

void ShooterSubsystem::assignOutput() {
	while (!ds->IsEnabled()) {;}
	subsystemHome();
	//cout << "Vout1,Amps1,Vout2,Amps2,Rpm" << endl;
	while(runThread)
	{
		shooterThreadControlStart = Timer::GetFPGATimestamp();

		if (ds->IsOperatorControl()) {
			if(buttonBox1->GetRawButton(SHOOT_BUTTON)) {
				shooterWheel->Enable();
				shooterWheel->Set(shooterVelocity);
			}
			else if (buttonBox1->GetRawButton(SHOOTER_OFF_BUTTON)) {
				shooterWheel->Disable();
				shooterWheel->Set(0);
			}

			if (buttonBox1->GetRawButton(SHOOT_FROM_HOPPER)) {
				selectShot(ShotSelection::hopper);
			} else if (buttonBox1->GetRawButton(SHOOT_FROM_LOADING_ZONE)) {
				selectShot(ShotSelection::loadingZone);
			} else if (buttonBox1->GetRawButton(SHOOT_FROM_CENTER_GEAR)) {
				selectShot(ShotSelection::gearCenter);
			} else if (buttonBox1->GetRawButton(SHOOT_FROM_SIDE_GEAR)) {
				selectShot(ShotSelection::gearFar);
			}

			if (armJoystick->GetRawButton(3)) {
				shooterVelocity += 25;
				shooterWheel->Set(shooterVelocity);
				this_thread::sleep_for(chrono::milliseconds(200));
			} else if (armJoystick->GetRawButton(4)) {
				shooterVelocity -= 25;
				shooterWheel->Set(shooterVelocity);
				this_thread::sleep_for(chrono::milliseconds(200));
			}

			if (armJoystick->GetRawButton(6)) {
				hoodMotor->Set(hoodMotor->GetPosition() + 0.05);
				this_thread::sleep_for(chrono::milliseconds(200));
			} else if (armJoystick->GetRawButton(5)) {
				hoodMotor->Set(hoodMotor->GetPosition() - 0.05);
				this_thread::sleep_for(chrono::milliseconds(200));
			}
		} else if (ds->IsAutonomous()) {
			this_thread::sleep_for(chrono::milliseconds(2000));
			selectShot(ShotSelection::hopper);
			while (ds->IsAutonomous()) {this_thread::sleep_for(chrono::milliseconds(20));;}
		}

		do {
			shooterThreadControlEnd = Timer::GetFPGATimestamp();
			shooterThreadControlElapsedTimeMS = (int) ((shooterThreadControlEnd - shooterThreadControlStart) * 1000);
			if (shooterThreadControlElapsedTimeMS < MIN_SHOOTER_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_SHOOTER_LOOP_TIME - shooterThreadControlElapsedTimeMS));
		} while(shooterThreadControlElapsedTimeMS < MIN_SHOOTER_LOOP_TIME);
	}
}

bool ShooterSubsystem::isAtSpeed() {
	return (abs(shooterWheel->GetSpeed() - shooterVelocity) < SPEED_THRESHOLD);
}

void ShooterSubsystem::setSpeed(double shooterVelocity) {
	this->shooterVelocity = shooterVelocity;
}

void ShooterSubsystem::subsystemHome() {
	hoodMotor->ConfigSoftPositionLimits(HOOD_UPPER_SOFT_STOP, -1);
	hoodMotor->SetControlMode(CANTalon::kPercentVbus);
	hoodMotor->Set(-0.12);
	this_thread::sleep_for(chrono::milliseconds(500));
	while (abs(hoodMotor->GetEncVel()) > 50) {cout << hoodMotor->GetEncVel() << endl;}
	cout << hoodMotor->GetEncVel() << endl;
	hoodMotor->Set(0);
	hoodMotor->SetEncPosition(0);
	hoodMotor->Disable();
	init();
	hoodMotor->ConfigSoftPositionLimits(HOOD_UPPER_SOFT_STOP, hoodMotor->GetPosition() + 0.05);
	hoodMotor->Enable();
	hoodMotor->Set(hoodMotor->GetPosition() + 0.05);
	cout << "Finished Shooter Home!" << endl;
}

void ShooterSubsystem::selectShot(ShotSelection shotSelection) {
	if (ds->GetAlliance() == DriverStation::Alliance::kRed) {
		switch (shotSelection) {
			case ShotSelection::hopper:
				shooterVelocity = -3250;
				shooterWheel->Set(shooterVelocity);
				hoodMotor->Set(0.045);
				break;
			case ShotSelection::loadingZone:
				shooterVelocity = -3650;
				shooterWheel->Set(shooterVelocity);
				hoodMotor->Set(0.216);
				break;
			case ShotSelection::gearCenter:
				shooterVelocity = -3550;
				shooterWheel->Set(shooterVelocity);
				hoodMotor->Set(0.216);
				break;
			case ShotSelection::gearFar:
				shooterVelocity = -3800;
				shooterWheel->Set(shooterVelocity);
				hoodMotor->Set(0.256);
				break;
			default:
				break;
		}
	} else {
		switch (shotSelection) {
			case ShotSelection::hopper:
				shooterVelocity = -3000;
				shooterWheel->Set(shooterVelocity);
				hoodMotor->Set(0);
				break;
			case ShotSelection::loadingZone:
				shooterVelocity = -3850;
				shooterWheel->Set(shooterVelocity);
				hoodMotor->Set(0.398);
				break;
			case ShotSelection::gearCenter:
				shooterVelocity = -3850;
				shooterWheel->Set(shooterVelocity);
				hoodMotor->Set(0.398);
				break;
			case ShotSelection::gearFar:
				shooterVelocity = -3650;
				shooterWheel->Set(shooterVelocity);
				hoodMotor->Set(0.198);
				break;
			default:
				break;
		}
	}
}

void ShooterSubsystem::stop() {
	runThread = false;
	if (shooterThread.joinable())
		shooterThread.join();
}
