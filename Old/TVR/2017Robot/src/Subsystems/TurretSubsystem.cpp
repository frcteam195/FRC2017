#include "Subsystems/TurretSubsystem.h"

using namespace frc;

TurretSubsystem::TurretSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
: TuneablePID("Turret", robotControllers->getTurretMotor(), UDP_PORT_NUMBER, false, false) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	turretMotor = robotControllers->getTurretMotor();
	turretSwitch = robotControllers->getTurretSwitch();

	runThread = false;
	homing = false;

	turretThreadControlElapsedTimeMS = 0;
	turretThreadControlStart = 0;
	turretThreadControlEnd = 0;

	timeoutStart = 0;
	timeoutEnd = 0;
	timeoutElapsedTimeMS = 0;

	turretPos = 0;

	prevVelocity = 0;
	prevAccel = 0;
}

TurretSubsystem::~TurretSubsystem() {}

void TurretSubsystem::init() {
	//turretMotor->SetMotionMagicCruiseVelocity(1200 * 0.66666);
	//turretMotor->SetMotionMagicAcceleration(1200 * 2);

	turretMotor->SetMotionMagicCruiseVelocity(1200);
	turretMotor->SetMotionMagicAcceleration(2800);
	turretMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	turretMotor->SetSensorDirection(false);
	turretMotor->SetPID(4, 0, 60, 0.1248779297);
	turretMotor->SetTalonControlMode(CANTalon::kMotionMagicMode);
	/*turretMotor->SetControlMode(CANTalon::kPercentVbus);
	turretMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);*/

	turretMotor->ConfigSoftPositionLimits(TURRET_UPPER_SOFT_STOP, TURRET_LOWER_SOFT_STOP);
	turretMotor->SetPosition(0.734);
}

void TurretSubsystem::start() {
	runThread = true;
	turretSubsystemThread = thread(&TurretSubsystem::runTurret, this);
}

void TurretSubsystem::subsystemHome() {
	homing = true;
	turretMotor->Disable();
	turretMotor->SetPosition(0);
	this_thread::sleep_for(chrono::milliseconds(25));
	turretMotor->Enable();
	turretMotor->Set(0);
	homing = false;
}

void TurretSubsystem::homeTurret() {
	homing = true;
	turretMotor->SetControlMode(CANTalon::kPercentVbus);
	timeoutStart = Timer::GetFPGATimestamp();
	if (turretSwitch->Get()) {
		while (turretSwitch->Get() && timeoutElapsedTimeMS < TURRET_HOME_TIMEOUT_MS) {
			turretMotor->Set(0.1);
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}
	} else {
		while (!turretSwitch->Get() && timeoutElapsedTimeMS < TURRET_HOME_TIMEOUT_MS) {
			turretMotor->Set(-0.1);
			this_thread::sleep_for(chrono::milliseconds(25));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
		}
	}
	turretMotor->Disable();
	init();
	this_thread::sleep_for(chrono::milliseconds(25));
	turretMotor->Enable();
	turretMotor->Set(0);
	homing = false;
}

void TurretSubsystem::stop() {
	runThread = false;
	if(turretSubsystemThread.joinable()) {
		turretSubsystemThread.join();
	}
}

void TurretSubsystem::runTurret() {
	while(!ds->IsEnabled()) {;}
	subsystemHome();

	while(runThread) {
		turretThreadControlStart = Timer::GetFPGATimestamp();

		if (!homing)
			turretMotor->Set(turretPos);

		do {
			turretThreadControlEnd = Timer::GetFPGATimestamp();
			turretThreadControlElapsedTimeMS = (int) ((turretThreadControlEnd - turretThreadControlStart) * 1000);
			if (turretThreadControlElapsedTimeMS < MIN_TURRET_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_TURRET_LOOP_TIME - turretThreadControlElapsedTimeMS));
		} while(turretThreadControlElapsedTimeMS < MIN_TURRET_LOOP_TIME);
	}
}

void TurretSubsystem::selectShot(ShotSelection shotSelection) {
	_subsystemMutex.lock();
	turretMotor->SetMotionMagicCruiseVelocity(1200);
	turretMotor->SetMotionMagicAcceleration(9000);
	if (ds->GetAlliance() == DriverStation::Alliance::kRed) {
		switch (shotSelection) {
			case ShotSelection::hopper:
				turretPos = 1.46;
				break;
			case ShotSelection::loadingZone:
				turretPos = -1.291;
				break;
			case ShotSelection::gearCenter:
				turretPos = -0.994;
				break;
			case ShotSelection::gearFar:
				turretPos = 0.459;
				break;
			case ShotSelection::removeBalls:
				turretPos = -1.717;
				break;
			default:
				break;
		}
	} else {
		switch (shotSelection) {
		case ShotSelection::hopper:
			turretPos = -1.43;
			break;
		case ShotSelection::loadingZone:
			turretPos = 1.291;
			break;
		case ShotSelection::gearCenter:
			turretPos = 0.994;
			break;
		case ShotSelection::gearFar:
			turretPos = -0.465;
			break;
		case ShotSelection::removeBalls:
			turretPos = 1.717;
			break;
		default:
			break;
		}
	}
	_subsystemMutex.unlock();
}

double TurretSubsystem::getTurretPos() {
	return turretMotor->GetPosition();
}

double TurretSubsystem::getTurretSetpoint() {
	return turretPos;
}

void TurretSubsystem::setTurretPos(double turretPos) {
	_subsystemMutex.lock();
	this->turretPos = turretPos;
	_subsystemMutex.unlock();
}

void TurretSubsystem::configVelocityAccel(double velocity, double accel) {
	if (prevVelocity != velocity)
		turretMotor->SetMotionMagicCruiseVelocity(velocity);

	if (prevAccel != accel)
		turretMotor->SetMotionMagicAcceleration(accel);

	prevVelocity = velocity;
	prevAccel = accel;
}
