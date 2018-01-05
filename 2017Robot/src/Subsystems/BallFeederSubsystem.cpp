#include "Subsystems/BallFeederSubsystem.h"

BallFeederSubsystem::BallFeederSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	carouselMotor = robotControllers->getCarouselMotor();
	feederMotor = robotControllers->getFeederMotor();

	runThread = false;

	ballFeederThreadControlStart = 0;
	ballFeederThreadControlEnd = 0;
	ballFeederThreadControlElapsedTimeMS = 0;

	carouselMotorSpeed = 0;
	feederMotorSpeed = 0;
}

BallFeederSubsystem::~BallFeederSubsystem() {}

void BallFeederSubsystem::init() {
	carouselMotor->SetControlMode(CANTalon::kPercentVbus);
	feederMotor->SetControlMode(CANTalon::kPercentVbus);
}

void BallFeederSubsystem::start() {
	runThread = true;
	ballFeederThread = thread(&BallFeederSubsystem::runBallFeeder, this);
}

void BallFeederSubsystem::subsystemHome() {
	;
}

void BallFeederSubsystem::setFeeder(double carouselMotorSpeed, double feederMotorSpeed) {
	_subsystemMutex.lock();
	carouselMotorSpeed = carouselMotorSpeed > MAX_SPEED ? MAX_SPEED : carouselMotorSpeed;
	carouselMotorSpeed = carouselMotorSpeed < MIN_SPEED ? MIN_SPEED : carouselMotorSpeed;
	this->carouselMotorSpeed = carouselMotorSpeed;
	feederMotorSpeed = feederMotorSpeed > MAX_SPEED ? MAX_SPEED : feederMotorSpeed;
	feederMotorSpeed = feederMotorSpeed < MIN_SPEED ? MIN_SPEED : feederMotorSpeed;
	this->feederMotorSpeed = feederMotorSpeed;
	_subsystemMutex.unlock();
}

void BallFeederSubsystem::stop() {
	runThread = false;

	if(ballFeederThread.joinable())
		ballFeederThread.join();
}

void BallFeederSubsystem::runBallFeeder() {
	while (!ds->IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(20));}
	subsystemHome();

	while(runThread) {
		ballFeederThreadControlStart = Timer::GetFPGATimestamp();

		carouselMotor->Set(carouselMotorSpeed);
		feederMotor->Set(feederMotorSpeed);

		do {
			ballFeederThreadControlEnd = Timer::GetFPGATimestamp();
			ballFeederThreadControlElapsedTimeMS = (int) ((ballFeederThreadControlEnd - ballFeederThreadControlStart) * 1000);
			if (ballFeederThreadControlElapsedTimeMS < MIN_FEEDER_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_FEEDER_LOOP_TIME - ballFeederThreadControlElapsedTimeMS));
		} while(ballFeederThreadControlElapsedTimeMS < MIN_FEEDER_LOOP_TIME);
	}
}

void BallFeederSubsystem::setFeederOn() {
	setFeeder(-0.75,-1);
}

void BallFeederSubsystem::setFeederOff() {
	setFeeder(0,0);
}
