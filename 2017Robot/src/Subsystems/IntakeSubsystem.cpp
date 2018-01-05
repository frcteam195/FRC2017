#include "Subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	intakeMotor = robotControllers->getIntakeMotor();
	intakeMotor2 = robotControllers->getIntakeMotor2();

	intakeSol = robotControllers->getIntakeSol();
	releaseSol = robotControllers->getReleaseSol();

	intakeSol->Set(DoubleSolenoid::kReverse);
	releaseSol->Set(false);

	runThread = false;

	intakeThreadControlStart = 0;
	intakeThreadControlEnd = 0;
	intakeThreadControlElapsedTimeMS = 0;

	climberDeployed = false;

	intakeSpeed = 0;
}

IntakeSubsystem::~IntakeSubsystem() {}

void IntakeSubsystem::init() {
	intakeMotor->SetControlMode(CANTalon::kPercentVbus);
	intakeMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);

	intakeMotor2->SetControlMode(CANTalon::kFollower);
	intakeMotor2->Set(intakeMotor->GetDeviceID());
}

void IntakeSubsystem::start() {
	runThread = true;
	intakeThread = thread(&IntakeSubsystem::runIntake, this);
}

void IntakeSubsystem::subsystemHome() {
	;
}

void IntakeSubsystem::stop() {
	runThread = false;
	if(intakeThread.joinable())
		intakeThread.join();
}

void IntakeSubsystem::runIntake() {
	while (!ds->IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(20));}
	subsystemHome();

	while(runThread) {
		intakeThreadControlStart = Timer::GetFPGATimestamp();

		if (!climberDeployed)
			intakeMotor->Set(intakeSpeed);
		else {
			if (intakeSpeed <= 0)
				intakeMotor->Set(intakeSpeed);
			else
				intakeMotor->Set(0);
		}

		do {
			intakeThreadControlEnd = Timer::GetFPGATimestamp();
			intakeThreadControlElapsedTimeMS = (int) ((intakeThreadControlEnd - intakeThreadControlStart) * 1000);
			if (intakeThreadControlElapsedTimeMS < MIN_INTAKE_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_INTAKE_LOOP_TIME - intakeThreadControlElapsedTimeMS));
		} while(intakeThreadControlElapsedTimeMS < MIN_INTAKE_LOOP_TIME);
	}
}

void IntakeSubsystem::setIntakeSpeed(double speed) {
	_subsystemMutex.lock();
	intakeSpeed = speed;
	_subsystemMutex.unlock();
}

void IntakeSubsystem::setIntakePosition(bool out) {
	_intakePositionMutex.lock();
	if(releaseSol->Get() == DoubleSolenoid::kForward && !climberDeployed) {
		releaseSol->Set(false);
		this_thread::sleep_for(chrono::milliseconds(150));
	}

	if(out) {
		if (!climberDeployed)
			intakeSol->Set(DoubleSolenoid::kForward);
	}
	else {
		intakeSol->Set(DoubleSolenoid::kReverse);
	}
	_intakePositionMutex.unlock();
}

void IntakeSubsystem::setIntakeLock(bool lock) {
	_climberLockMutex.lock();
	if (lock) {
		if (intakeSol->Get() == DoubleSolenoid::kReverse) {
			releaseSol->Set(true);
		}
		climberDeployed = true;
	}
	else
		if (!climberDeployed)
			releaseSol->Set(false);
	_climberLockMutex.unlock();
}

bool IntakeSubsystem::isIntakeOut() {
	return (intakeSol->Get() == DoubleSolenoid::kForward);
}

bool IntakeSubsystem::isIntakeLocked() {
	return (releaseSol->Get() == DoubleSolenoid::kForward);
}
