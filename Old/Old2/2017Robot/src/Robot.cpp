#include "Robot.h"

void Robot::RobotInit() {
	counter = 0;

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

	autoHopperRed = new AutoHopperRed(robotControllers, &subsystemVector);

	for(unsigned int i = 0; i < subsystemVector.size(); i++)
		subsystemVector.at(i)->init();

	for (unsigned int i = 0; i < subsystemVector.size(); i++)
		subsystemVector.at(i)->start();
}

void Robot::Autonomous() {
	cout << "started auto" << endl;

	autoHopperRed->start();

	while(!IsOperatorControl() && IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(100));}
	robotDrive->changeControlMode(CANSpeedController::kPercentVbus);
	cout << "auto End" << endl;
}

void Robot::OperatorControl() {
	cout << "started OC" << endl;
	robotDrive->changeControlMode(CANSpeedController::kPercentVbus);
	while(IsOperatorControl() && IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(100));}

	cout << "OC End" << endl;
}

void Robot::Test() {

}

START_ROBOT_CLASS(Robot)
