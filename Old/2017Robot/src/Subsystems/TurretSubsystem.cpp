#include "Subsystems/TurretSubsystem.h"

using namespace frc;

TurretSubsystem::TurretSubsystem(VisionReceiverSubsystem *visionReciever, Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
: TuneablePID("Turret", robotControllers->getTurretMotor(), UDP_PORT_NUMBER, false, false) {
	subsystemVector->push_back(this);

	ds = &DriverStation::GetInstance();

	this->visionReceiver = visionReciever;

	armJoystick = robotControllers->getArmJoystick();
	buttonBox1 = robotControllers->getButtonBox1();

	turretMotor = robotControllers->getTurretMotor();

	driveSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(robotControllers->getDriveSubsystemIndex()));

	deviation = 0.0;

	visionData = new VisionData();

	runThread = false;

	zAxis = 0;

	ctrlMode = TurretControlMode::predictiveSeek;

	turretThreadControlElapsedTimeMS = 0;
	turretThreadControlStart = 0;
	turretThreadControlEnd = 0;

	axisControlStart = 0;
	axisControlEnd = 0;
	axisControlElapsedTimeMS = 0;

	filteredOnTarget = false;
	filterCounter = 0;
	prevOnTargetVal = false;
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
	turretMotor->SetEncPosition(0);

	/*turretMotor->SetControlMode(CANTalon::kPercentVbus);
	turretMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);*/

	turretMotor->ConfigSoftPositionLimits(TURRET_UPPER_SOFT_STOP, TURRET_LOWER_SOFT_STOP);
	turretMotor->SetEncPosition(0);
}

void TurretSubsystem::start() {
	runThread = true;
	turretSubsystemThread = thread(&TurretSubsystem::runTurret, this);
}

void TurretSubsystem::subsystemHome() {
	turretMotor->SetEncPosition(0);
	turretMotor->Set(0);
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

		visionData = visionReceiver->getVisionData();

		if (ds->IsOperatorControl()) {


			if (armJoystick->GetRawButton(MANUAL_OVERRIDE_BUTTON)) {
				ctrlMode = TurretControlMode::manualSetpoint;
				turretMotor->SetMotionMagicCruiseVelocity(1200);
				turretMotor->SetMotionMagicAcceleration(1200);
			}
			else if (visionData->targetFound) {
				ctrlMode = TurretControlMode::vision;
				turretMotor->SetMotionMagicCruiseVelocity(1200);
				turretMotor->SetMotionMagicAcceleration(2800);
			}
			else if (armJoystick->GetRawButton(PREDICTIVE_SEEK_BUTTON)) {
				ctrlMode = TurretControlMode::predictiveSeek;
				turretMotor->SetMotionMagicCruiseVelocity(1200);
				turretMotor->SetMotionMagicAcceleration(2800);
			}
			if (buttonBox1->GetRawButton(SHOOT_FROM_HOPPER)) {
				if (ds->GetAlliance() == DriverStation::Alliance::kRed) {
					turretMotor->Set(1.46);
					//turretMotor->Set(1.37);
				} else {
					turretMotor->Set(-1.432);
					//turretMotor->Set(1.37);
				}
				this_thread::sleep_for(chrono::milliseconds(500));
			} else if (ctrlMode == TurretControlMode::vision) {
				if(armJoystick->GetRawButton(1)) {
					if(!visionData->onTarget) {
						if (visionData->targetFound) {
							deviation = visionReceiver->getVisionData()->angleDeviation * 124 / 18 / 360;
							if(turretMotor->GetPosition() + deviation > TURRET_LOWER_SOFT_STOP && turretMotor->GetPosition() + deviation < TURRET_UPPER_SOFT_STOP)
								turretMotor->Set(turretMotor->GetPosition() + deviation);
						}
						else {
							//TODO: Add code to predict boiler position when target is not found
						}
					}
				}
			} else if (ctrlMode == TurretControlMode::manualSetpoint) {

				axisControlStart = Timer::GetFPGATimestamp();

				zAxis = armJoystick->GetRawAxis(MANUAL_CONTROL_AXIS);
				zAxis = abs(zAxis) < TURRET_JOYSTICK_DEADBAND ? 0 : zAxis;
				if (zAxis != 0) {
					zAxis = sgn(zAxis) * ((abs(zAxis) - TURRET_JOYSTICK_DEADBAND) / (1 - TURRET_JOYSTICK_DEADBAND));
					zAxis *= MANUAL_CONTROL_STEP;
					turretMotor->Set(turretMotor->GetPosition() + zAxis);
				}

				do {
					axisControlEnd = Timer::GetFPGATimestamp();
					axisControlElapsedTimeMS = (int) ((axisControlEnd - axisControlStart) * 1000);
					if (axisControlElapsedTimeMS < MANUAL_TIME_STEP_MS)
						this_thread::sleep_for(chrono::milliseconds(MANUAL_TIME_STEP_MS - axisControlElapsedTimeMS));
				} while(axisControlElapsedTimeMS < MANUAL_TIME_STEP_MS);
			} else if (ctrlMode == TurretControlMode::predictiveSeek) {

			} else {

			}
		} else if (ds->IsAutonomous()) {
			if (ds->GetAlliance() == DriverStation::Alliance::kRed) {
				turretMotor->Set(1.46);
				//turretMotor->Set(1.37);
			} else {
				turretMotor->Set(-1.432);
				//turretMotor->Set(1.37);
			}

			while (!driveSubsystem->isAutoDriveFinished()) {this_thread::sleep_for(chrono::milliseconds(20));}

			while (ds->IsAutonomous() && !filteredOnTarget) {
					deviation = visionReceiver->getVisionData()->angleDeviation * 124 / 18 / 360;
					if(turretMotor->GetPosition() + deviation > TURRET_LOWER_SOFT_STOP && turretMotor->GetPosition() + deviation < TURRET_UPPER_SOFT_STOP)
						turretMotor->Set(turretMotor->GetPosition() + deviation);

					if (!filteredOnTarget) {
						prevOnTargetVal &= visionReceiver->getVisionData()->onTarget;
						this_thread::sleep_for(chrono::milliseconds(20));
						if (prevOnTargetVal && filterCounter > 10)
							filteredOnTarget = true;
					}
			}

			while (ds->IsAutonomous()) {this_thread::sleep_for(chrono::milliseconds(20));}
		}

		do {
			turretThreadControlEnd = Timer::GetFPGATimestamp();
			turretThreadControlElapsedTimeMS = (int) ((turretThreadControlEnd - turretThreadControlStart) * 1000);
			if (turretThreadControlElapsedTimeMS < MIN_TURRET_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_TURRET_LOOP_TIME - turretThreadControlElapsedTimeMS));
		} while(turretThreadControlElapsedTimeMS < MIN_TURRET_LOOP_TIME);
	}
}

bool TurretSubsystem::isOnTarget() {
	return filteredOnTarget;
}

double TurretSubsystem::sgn(double x) {
    return (x > 0) - (x < 0);
}
