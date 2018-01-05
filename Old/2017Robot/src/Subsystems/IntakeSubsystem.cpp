#include "Subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	buttonBox1 = robotControllers->getButtonBox1();
	buttonBox2 = robotControllers->getButtonBox2();

	intakeMotor = robotControllers->getIntakeMotor();
	intakeMotor2 = robotControllers->getIntakeMotor2();

	intakeSol = robotControllers->getIntakeSol();
	releaseSol = robotControllers->getReleaseSol();

	intakeSol->Set(DoubleSolenoid::kReverse);
	releaseSol->Set(DoubleSolenoid::kReverse);

	runThread = false;

	intakeThreadControlStart = 0;
	intakeThreadControlEnd = 0;
	intakeThreadControlElapsedTimeMS = 0;

	climberDeployed = false;
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
	while (!ds->IsEnabled()) {;}
	subsystemHome();

	while(runThread) {
		intakeThreadControlStart = Timer::GetFPGATimestamp();

		if (ds->IsOperatorControl()) {


			if(buttonBox1->GetRawButton(INTAKE_IN_BUTTON))
				intakeMotor->Set(1);
			else if(buttonBox1->GetRawButton(INTAKE_OUT_BUTTON))
				intakeMotor->Set(-1);
			else if (buttonBox2->GetRawButton(CLIMB_BUTTON)) {
				 if (isIntakeOut()) {
					 setIntakePosition(false);
					 this_thread::sleep_for(chrono::milliseconds(1100));
				 }

				 if (!isIntakeLocked()) {
					 setIntakeLock(true);
					 this_thread::sleep_for(chrono::milliseconds(350));
				 }

				 climberDeployed = true;

				 intakeMotor->Set(-1);
			} else {
				intakeMotor->Set(0);
			}


			if(buttonBox1->GetRawButton(INTAKE_ACTUATE_OUT_BUTTON))
				setIntakePosition(true);
			else if(buttonBox1->GetRawButton(INTAKE_ACTUATE_IN_BUTTON))
				setIntakePosition(false);

			if(buttonBox1->GetRawButton(LOCK_INTAKE_BUTTON))
				setIntakeLock(true);
			else if(buttonBox1->GetRawButton(UNLOCK_INTAKE_BUTTON))
				setIntakeLock(false);
		} else if (ds->IsAutonomous()) {
			setIntakePosition(true);
			while (ds->IsAutonomous()) {this_thread::sleep_for(chrono::milliseconds(20));;}
		}


		do {
			intakeThreadControlEnd = Timer::GetFPGATimestamp();
			intakeThreadControlElapsedTimeMS = (int) ((intakeThreadControlEnd - intakeThreadControlStart) * 1000);
			if (intakeThreadControlElapsedTimeMS < MIN_INTAKE_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_INTAKE_LOOP_TIME - intakeThreadControlElapsedTimeMS));
		} while(intakeThreadControlElapsedTimeMS < MIN_INTAKE_LOOP_TIME);
	}
}

void IntakeSubsystem::setIntakePosition(bool out) {
	if (releaseSol->Get() == DoubleSolenoid::kForward && !climberDeployed) {
		releaseSol->Set(DoubleSolenoid::kReverse);
		this_thread::sleep_for(chrono::milliseconds(150));
	}

	if (out) {
		if (!climberDeployed)
			intakeSol->Set(DoubleSolenoid::kForward);
	}
	else {
		intakeSol->Set(DoubleSolenoid::kReverse);
	}
}

void IntakeSubsystem::setIntakeLock(bool lock) {
	if (lock) {
		if (intakeSol->Get() == DoubleSolenoid::kReverse) {
			releaseSol->Set(DoubleSolenoid::kForward);
		}
	}
	else
		if (!climberDeployed)
			releaseSol->Set(DoubleSolenoid::kReverse);
}

bool IntakeSubsystem::isIntakeOut() {
	return (intakeSol->Get() == DoubleSolenoid::kForward);
}

bool IntakeSubsystem::isIntakeLocked() {
	return (releaseSol->Get() == DoubleSolenoid::kForward);
}
