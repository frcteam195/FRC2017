#include "Subsystems/GearSubsystem.h"

GearSubsystem::GearSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
: TuneablePID("Gear", robotControllers->getGearActuator(), 5808, true, true) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	gearActuator = robotControllers->getGearActuator();

	gearClamp = robotControllers->getGearClamp();

	gearSwitch = robotControllers->getGearSwitch();
	gearPusher = robotControllers->getGearPusher();

	runThread = false;

	gearThreadControlStart = 0;
	gearThreadControlEnd = 0;
	gearThreadControlElapsedTimeMS = 0;

	gearActuatorPos = 0;
	gearClampOpen = false;
	gearPushed = false;

	homing = false;

	prevVelocity = 0;
	prevAccel = 0;
}

GearSubsystem::~GearSubsystem() {}

void GearSubsystem::init() {
	gearActuator->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	gearActuator->SetSensorDirection(false);
	gearActuator->SetPID(3, 0, 30, 1.1100260417);
	gearActuator->SetTalonControlMode(CANTalon::kMotionMagicMode);
	gearActuator->SetMotionMagicCruiseVelocity(50);
	gearActuator->SetMotionMagicAcceleration(150);
	gearActuator->SetPosition(0);
	gearActuator->ConfigSoftPositionLimits(GEAR_UPPER_SOFT_STOP, GEAR_LOWER_SOFT_STOP);
	}

void GearSubsystem::start() {
	runThread = true;
	gearSubsystemThread = thread(&GearSubsystem::runGearSubsystem, this);
}

void GearSubsystem::subsystemHome() {
	homing = true;
	gearClamp->Set(false);
	gearClampOpen = false;
	gearActuator->SetControlMode(CANTalon::kPercentVbus);
	gearActuator->Set(0.1);
	this_thread::sleep_for(chrono::milliseconds(300));
	while (abs(gearActuator->GetSpeed()) > 0.001) {;}
	gearActuator->Set(0);
	gearActuator->SetPosition(0);
	gearActuator->Disable();
	init();
	gearActuator->Enable();
	gearActuator->Set(POSITION_ZERO);
	homing = false;
}

void GearSubsystem::setGearActuatorPos(double pos) {
	_subsystemMutex.lock();
	gearActuatorPos = pos;
	_subsystemMutex.unlock();
}

void GearSubsystem::setGearClamp(bool open) {
	_gearClampMutex.lock();
	gearClampOpen = open;
	_gearClampMutex.unlock();
}

void GearSubsystem::setGearPusher(bool pushed) {
	_gearPusherMutex.lock();
	gearPushed = pushed;
	_gearPusherMutex.unlock();
}

void GearSubsystem::stop() {
	runThread = false;
	if(gearSubsystemThread.joinable())
		gearSubsystemThread.join();
}

void GearSubsystem::runGearSubsystem() {
	while (!ds->IsEnabled()) {;}
	subsystemHome();

	while(runThread) {
		gearThreadControlStart = Timer::GetFPGATimestamp();

		if (!homing) {
			gearActuator->Set(gearActuatorPos);
			gearClamp->Set(gearClampOpen);
			gearPusher->Set(gearPushed);
		}

		do {
			gearThreadControlEnd = Timer::GetFPGATimestamp();
			gearThreadControlElapsedTimeMS = (int) ((gearThreadControlEnd - gearThreadControlStart) * 1000);
			if (gearThreadControlElapsedTimeMS < MIN_GEAR_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_GEAR_LOOP_TIME - gearThreadControlElapsedTimeMS));
		} while(gearThreadControlElapsedTimeMS < MIN_GEAR_LOOP_TIME);
	}
}

double GearSubsystem::getGearActuatorPos() {
	return gearActuator->GetPosition();
}

void GearSubsystem::configVelocityAccel(double velocity, double accel) {
	if (prevVelocity != velocity)
		gearActuator->SetMotionMagicCruiseVelocity(velocity);

	if (prevAccel != accel)
		gearActuator->SetMotionMagicAcceleration(accel);

	prevVelocity = velocity;
	prevAccel = accel;
}
