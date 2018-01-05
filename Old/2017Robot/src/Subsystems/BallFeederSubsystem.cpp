#include "Subsystems/BallFeederSubsystem.h"

BallFeederSubsystem::BallFeederSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector) {
	subsystemVector->push_back(this);

	shooterSubsystem = dynamic_cast<ShooterSubsystem*>(subsystemVector->at(robotControllers->getShooterSubsystemIndex()));
	driveSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()));
	turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));

	ds = &DriverStation::GetInstance();

	buttonBox1 = robotControllers->getButtonBox1();
	buttonBox2 = robotControllers->getButtonBox2();

	carouselMotor = robotControllers->getCarouselMotor();
	feederMotor = robotControllers->getFeederMotor();

	runThread = false;

	ballFeederThreadControlStart = 0;
	ballFeederThreadControlEnd = 0;
	ballFeederThreadControlElapsedTimeMS = 0;
}

BallFeederSubsystem::~BallFeederSubsystem() {}

void BallFeederSubsystem::init() {}

void BallFeederSubsystem::start() {
	runThread = true;
	ballFeederThread = thread(&BallFeederSubsystem::runBallFeeder, this);
}

void BallFeederSubsystem::subsystemHome() {
	;
}

void BallFeederSubsystem::stop() {
	runThread = false;

	if(ballFeederThread.joinable())
		ballFeederThread.join();
}

void BallFeederSubsystem::runBallFeeder() {
	while (!ds->IsEnabled()) {;}
	subsystemHome();

	while(runThread) {
		ballFeederThreadControlStart = Timer::GetFPGATimestamp();

		if (ds->IsOperatorControl()) {
			if(buttonBox1->GetRawButton(SHOOT_BUTTON)) {
				if (shooterSubsystem->isAtSpeed()) {
					carouselMotor->Set(-1);
					feederMotor->Set(-1);
				}
			} else if(buttonBox2->GetRawButton(AGITATOR_REVERSE)) {
				carouselMotor->Set(1);
			} else if(buttonBox2->GetRawButton(FEEDER_REVERSE)) {
				feederMotor->Set(1);
			}
			else {
				carouselMotor->Set(0);
				feederMotor->Set(0);
			}
		} else if (ds->IsAutonomous()) {
			while (!driveSubsystem->isAutoDriveFinished() && !turretSubsystem->isOnTarget()) {this_thread::sleep_for(chrono::milliseconds(20));}
			carouselMotor->Set(-1);
			feederMotor->Set(-1);
			while (ds->IsAutonomous()) {this_thread::sleep_for(chrono::milliseconds(20));}
		}

		do {
			ballFeederThreadControlEnd = Timer::GetFPGATimestamp();
			ballFeederThreadControlElapsedTimeMS = (int) ((ballFeederThreadControlEnd - ballFeederThreadControlStart) * 1000);
			if (ballFeederThreadControlElapsedTimeMS < MIN_FEEDER_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_FEEDER_LOOP_TIME - ballFeederThreadControlElapsedTimeMS));
		} while(ballFeederThreadControlElapsedTimeMS < MIN_FEEDER_LOOP_TIME);
	}
}
