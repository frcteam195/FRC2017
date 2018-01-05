#include "Robot.h"

void Robot::RobotInit() {
	counter = 0;

	selectedAuto = AutoSelection::kHopper;

	robotControllers = new Controllers();

	cs = CameraServer::GetInstance();
	cs->StartAutomaticCapture(0);

	robotDrive = new DriveBaseSubsystem(robotControllers, &subsystemVector);
	robotControllers->setDriveSubsystemIndex(counter++);
	robotGearSubsystem = new GearSubsystem(robotControllers, &subsystemVector);
	robotControllers->setGearSubsystemIndex(counter++);
	robotIntake = new IntakeSubsystem(robotControllers, &subsystemVector);
	robotControllers->setIntakeSubsystemIndex(counter++);
	visionReceiver = new VisionReceiverSubsystem(5801, &subsystemVector);
	robotControllers->setVisionSubsystemIndex(counter++);
	shooterTurret = new TurretSubsystem(robotControllers, &subsystemVector);
	robotControllers->setTurretSubsystemIndex(counter++);
	ballShooter = new ShooterSubsystem(robotControllers, &subsystemVector);
	robotControllers->setShooterSubsystemIndex(counter++);
	robotBallFeeder = new BallFeederSubsystem(robotControllers, &subsystemVector);
	robotControllers->setFeederSubsystemIndex(counter++);

	//Placeholder for other subsystem instantiation

	hid = new HIDControllerSubsystem(robotControllers, &subsystemVector);
	autoSelector = new AutoSelectionUDPReceiver(5803, &subsystemVector);

	autoHopperRed = new AutoHopperRed(robotControllers, &subsystemVector);
	autoRedBoilerSideGear = new AutoRedBoilerSideGear(robotControllers, &subsystemVector);
	autoRedBoilerSideGearHopper = new AutoRedBoilerSideGearHopper(robotControllers, &subsystemVector);

	autoHopperBlue = new AutoHopperBlue(robotControllers, &subsystemVector);
	autoBlueBoilerSideGear = new AutoBlueBoilerSideGear(robotControllers, &subsystemVector);
	autoBlueBoilerSideGearHopper = new AutoBlueBoilerSideGearHopper(robotControllers, &subsystemVector);

	for(unsigned int i = 0; i < subsystemVector.size(); i++)
		subsystemVector.at(i)->init();

	for (unsigned int i = 0; i < subsystemVector.size(); i++)
		subsystemVector.at(i)->start();
}

void Robot::Autonomous() {
	selectedAuto = autoSelector->getAutoMode();
	autoSelector->stop();
	switch  (selectedAuto) {
		case AutoSelection::kHopper:
			if (DriverStation::GetInstance().GetAlliance() == DriverStation::kRed) {
				autoHopperRed->start();
			} else {
				autoHopperBlue->start();
			}
			break;
		case AutoSelection::kSideGearHopper:
			if (DriverStation::GetInstance().GetAlliance() == DriverStation::kRed) {
				autoRedBoilerSideGearHopper->start();
			} else {
				autoBlueBoilerSideGearHopper->start();
			}
			break;
		case AutoSelection::kSideGear:
			if (DriverStation::GetInstance().GetAlliance() == DriverStation::kRed) {
				autoRedBoilerSideGear->start();
			} else {
				autoBlueBoilerSideGear->start();
			}
			break;
		default:
			if (DriverStation::GetInstance().GetAlliance() == DriverStation::kRed) {
				autoHopperRed->start();
			} else {
				autoHopperBlue->start();
			}
			break;
	}

	while(!IsOperatorControl() && IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(100));}
	robotDrive->changeControlMode(CANSpeedController::kPercentVbus);
}

void Robot::OperatorControl() {
	robotDrive->changeControlMode(CANSpeedController::kPercentVbus);
	while(IsOperatorControl() && IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(100));}
}

void Robot::Test() {

}

START_ROBOT_CLASS(Robot)
