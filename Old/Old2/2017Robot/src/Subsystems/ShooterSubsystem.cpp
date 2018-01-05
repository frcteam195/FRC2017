#include "Subsystems/ShooterSubsystem.h"
#include <iostream>

using namespace std;
using namespace frc;

ShooterSubsystem::ShooterSubsystem(Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector)
: TuneablePID("Shooter", robotControllers->getShooterWheel(), UDP_PORT_NUMBER, true, true) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	shooterWheel = robotControllers->getShooterWheel();
	hoodMotor = robotControllers->getHoodMotor();

	shooterVelocity = 0;
	hoodPos = 0;

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

	hoodMotor->SetMotionMagicCruiseVelocity(1200);
	hoodMotor->SetMotionMagicAcceleration(9000);
	hoodMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	hoodMotor->SetSensorDirection(false);
	hoodMotor->SetPID(20, 0, 200, 1.1352539063);
	hoodMotor->SetTalonControlMode(CANTalon::kMotionMagicMode);
	hoodMotor->SetPosition(0);
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

		if (shooterVelocity == 0) {
			shooterWheel->Disable();
		} else {
			if (!shooterWheel->IsEnabled())
			shooterWheel->Enable();
		}
		shooterWheel->Set(shooterVelocity);
		hoodMotor->Set(hoodPos);

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
	_subsystemMutex.lock();
	this->shooterVelocity = shooterVelocity;
	_subsystemMutex.unlock();
}

void ShooterSubsystem::setHoodPos(double hoodPos) {
	_subsystemMutex.lock();
	this->hoodPos = hoodPos;
	_subsystemMutex.unlock();
}

void ShooterSubsystem::subsystemHome() {
	hoodMotor->ConfigSoftPositionLimits(HOOD_UPPER_SOFT_STOP, -1);
	hoodMotor->SetControlMode(CANTalon::kPercentVbus);
	hoodMotor->Set(-0.12);
	this_thread::sleep_for(chrono::milliseconds(100));
	while (abs(hoodMotor->GetSpeed()) > .002) {;}
	hoodMotor->Set(0);
	hoodMotor->SetPosition(0);
	hoodMotor->Disable();
	init();
	this_thread::sleep_for(chrono::milliseconds(25));
	hoodMotor->ConfigSoftPositionLimits(HOOD_UPPER_SOFT_STOP, hoodMotor->GetPosition() + 0.05);
	hoodMotor->Enable();
	hoodMotor->Set(hoodMotor->GetPosition() + 0.05);

}

void ShooterSubsystem::selectShot(ShotSelection shotSelection) {
	_subsystemMutex.lock();
	if (ds->GetAlliance() == DriverStation::Alliance::kRed) {
		switch (shotSelection) {
			case ShotSelection::hopper:
				//OldVals1
				//shooterVelocity = -3250;
				//hoodPos = 0.045;

				//OldVals2
				//shooterVelocity = -3125;
				//hoodPos = 0.095;

				//GinoVals
				//shooterVelocity = -3075;
				//hoodPos = 0.144;

				shooterVelocity = -3090;
				hoodPos = 0.153;
				break;
			case ShotSelection::loadingZone:
				shooterVelocity = -3750;
				hoodPos = 0.266;
				break;
			case ShotSelection::gearCenter:
				shooterVelocity = -3700;
				hoodPos = 0.216;
				break;
			case ShotSelection::gearFar:
				shooterVelocity = -3575;
				hoodPos = 0.312;
				break;
			case ShotSelection::removeBalls:
				shooterVelocity = -750;
				hoodPos = 0;
				break;
			default:
				break;
		}
	} else {
		switch (shotSelection) {
			case ShotSelection::hopper:
				shooterVelocity = -3000;
				hoodPos = 0;
				break;
			case ShotSelection::loadingZone:
				shooterVelocity = -3850;
				hoodPos = 0.398;
				break;
			case ShotSelection::gearCenter:
				shooterVelocity = -3850;
				hoodPos = 0.398;
				break;
			case ShotSelection::gearFar:
				shooterVelocity = -3650;
				hoodPos = 0.198;
				break;
			case ShotSelection::removeBalls:
				shooterVelocity = -750;
				hoodPos = 0;
				break;
			default:
				break;
		}
	}
	_subsystemMutex.unlock();
}

void ShooterSubsystem::bumpShooterSpeed(bool increase) {
	_subsystemMutex.lock();
	if (increase)
		shooterVelocity -= 25;
	else
		shooterVelocity += 25;
	_subsystemMutex.unlock();
}

void ShooterSubsystem::bumpHood(bool increase) {
	_subsystemMutex.lock();
	if (increase)
		hoodPos += 0.05;
	else
		hoodPos -= 0.05;
	_subsystemMutex.unlock();
}

void ShooterSubsystem::stop() {
	runThread = false;
	if (shooterThread.joinable())
		shooterThread.join();
}
