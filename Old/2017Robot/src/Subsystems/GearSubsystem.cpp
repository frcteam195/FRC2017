#include "Subsystems/GearSubsystem.h"

GearSubsystem::GearSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
: TuneablePID("Gear", robotControllers->getGearActuator(), 5808, true, true) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	armJoystick = robotControllers->getButtonBox1();
	driveJoystick = robotControllers->getDriveJoystick();

	gearActuator = robotControllers->getGearActuator();

	gearClamp = robotControllers->getGearClamp();

	runThread = false;

	gearThreadControlStart = 0;
	gearThreadControlEnd = 0;
	gearThreadControlElapsedTimeMS = 0;

	timeoutStart = 0;
	timeoutEnd = 0;
	timeoutElapsedTimeMS = 0;
}

GearSubsystem::~GearSubsystem() {}

void GearSubsystem::init() {
	gearActuator->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	gearActuator->SetSensorDirection(false);
	gearActuator->SetPID(3, 0, 30, 1.1100260417);
	gearActuator->SetTalonControlMode(CANTalon::kMotionMagicMode);
	gearActuator->SetMotionMagicCruiseVelocity(50);
	gearActuator->SetMotionMagicAcceleration(150);
	gearActuator->SetEncPosition(0);
	gearActuator->ConfigSoftPositionLimits(GEAR_UPPER_SOFT_STOP, GEAR_LOWER_SOFT_STOP);
	ds->ReportWarning("Finished Init for Gear!");
}

void GearSubsystem::start() {
	runThread = true;
	gearSubsystemThread = thread(&GearSubsystem::runGearSubsystem, this);
}

void GearSubsystem::subsystemHome() {
	gearActuator->SetControlMode(CANTalon::kPercentVbus);
	gearActuator->Set(0.1);
	this_thread::sleep_for(chrono::milliseconds(300));
	while (abs(gearActuator->GetEncVel()) > 10) {cout << gearActuator->GetEncVel() << endl;}
	cout << gearActuator->GetEncVel() << endl;
	gearActuator->Set(0);
	gearActuator->SetEncPosition(0);
	gearActuator->Disable();
	init();
	gearActuator->Enable();
	gearActuator->Set(POSITION_ZERO);
	cout << "Finished Home!" << endl;
	ds->ReportWarning("Finished Home for Gear!");
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

/*
		if(armJoystick->GetRawButton(GEAR_SUBSYSTEM_ACTUATE_DOWN))
			gearActuator->Set(POSITION_DOWN);
		else if(armJoystick->GetRawButton(GEAR_SUBSYSTEM_ACTUATE_UP))
			gearActuator->Set(POSITION_ZERO);

		if(armJoystick->GetRawButton(GEAR_SUBSYSTEM_OPEN_CLAMP))
			gearClamp->Set(true);
		else if(armJoystick->GetRawButton(GEAR_SUBSYSTEM_CLOSE_CLAMP))
			gearClamp->Set(false);*/

		if(armJoystick->GetRawButton(PICKUP_GEAR) || driveJoystick->GetRawButton(DRIVER_PICKUP_GEAR)) {
			gearActuator->Set(POSITION_ZERO);
			gearClamp->Set(true);
			this_thread::sleep_for(chrono::milliseconds(75));
			gearActuator->Set(POSITION_DOWN);

			timeoutElapsedTimeMS = 0;

			timeoutStart = Timer::GetFPGATimestamp();

			while (abs(gearActuator->GetPosition() - POSITION_DOWN) > DEVIATION_THRESHOLD && timeoutElapsedTimeMS < GEAR_DOWN_ACTION_TIMEOUT_MS) {
				this_thread::sleep_for(chrono::milliseconds(25));
				timeoutEnd = Timer::GetFPGATimestamp();
				timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
			}

			this_thread::sleep_for(chrono::milliseconds(350));

			gearClamp->Set(false);

			this_thread::sleep_for(chrono::milliseconds(75));

			gearActuator->Set(POSITION_HOLD);
		} else if(armJoystick->GetRawButton(PLACE_GEAR) || driveJoystick->GetRawButton(DRIVER_PLACE_GEAR)) {
			gearClamp->Set(true);
			this_thread::sleep_for(chrono::milliseconds(250));
			gearActuator->Set(POSITION_UP);
			this_thread::sleep_for(chrono::milliseconds(250));
			gearActuator->Set(POSITION_RELEASE);
			this_thread::sleep_for(chrono::milliseconds(250));
			gearClamp->Set(false);
			this_thread::sleep_for(chrono::milliseconds(250));
			gearClamp->Set(true);
			this_thread::sleep_for(chrono::milliseconds(250));
			gearActuator->Set(POSITION_ZERO);
		}

		do {
			gearThreadControlEnd = Timer::GetFPGATimestamp();
			gearThreadControlElapsedTimeMS = (int) ((gearThreadControlEnd - gearThreadControlStart) * 1000);
			if (gearThreadControlElapsedTimeMS < MIN_GEAR_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_GEAR_LOOP_TIME - gearThreadControlElapsedTimeMS));
		} while(gearThreadControlElapsedTimeMS < MIN_GEAR_LOOP_TIME);
	}
}
