#include <Subsystems/DriveBaseSubsystem.h>

using namespace std;
using namespace frc;

DriveBaseSubsystem::DriveBaseSubsystem(Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector) {
	ds = &DriverStation::GetInstance();
	subsystemVector->push_back(this);

	driveJoystick = robotControllers->getDriveJoystick();

	leftDrive = robotControllers->getLeftDrive1();
	leftDriveSlave1 = robotControllers->getLeftDrive2();
	leftDriveSlave2 = robotControllers->getLeftDrive3();
	rightDrive = robotControllers->getRightDrive1();
	rightDriveSlave1 = robotControllers->getRightDrive2();
	rightDriveSlave2 = robotControllers->getRightDrive3();

	shiftSol = robotControllers->getShiftSol();

	navX = robotControllers->getNavX();
	navX->ZeroYaw();

	x = 0.0;
	y = 0.0;
	left = 0.0;
	right = 0.0;
	absLeft = 0.0;
	absRight = 0.0;
	normalLeft = 0.0;
	normalRight = 0.0;

	shiftSol->Set(DoubleSolenoid::kReverse);
	driveGear = DoubleSolenoid::kReverse;
	highGear = false;
	holdLow = false;

	runThread = false;

	driveThreadControlStart = 0;
	driveThreadControlEnd = 0;
	driveThreadControlElapsedTimeMS = 0;

	shiftThreadControlStart = 0;
	shiftThreadControlEnd = 0;
	shiftThreadControlElapsedTimeMS = 0;

	currentValTmp = 0;
	avgPosTmp = 0;

	autoDriveFinished = false;

	angleController = new KnightPIDController(0, 0, 0, 0);
}

DriveBaseSubsystem::~DriveBaseSubsystem() {}

void DriveBaseSubsystem::init() {
	leftDriveSlave1->SetControlMode(CANTalon::kFollower);
	leftDriveSlave1->Set(leftDrive->GetDeviceID());
	leftDriveSlave2->SetControlMode(CANTalon::kFollower);
	leftDriveSlave2->Set(leftDrive->GetDeviceID());

	rightDriveSlave1->SetControlMode(CANTalon::kFollower);
	rightDriveSlave1->Set(rightDrive->GetDeviceID());
	rightDriveSlave2->SetControlMode(CANTalon::kFollower);
	rightDriveSlave2->Set(rightDrive->GetDeviceID());

	leftDrive->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	rightDrive->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);

	leftDrive->SetStatusFrameRateMs(CANTalon::StatusFrameRateQuadEncoder, 5);
	rightDrive->SetStatusFrameRateMs(CANTalon::StatusFrameRateQuadEncoder, 5);
}

void DriveBaseSubsystem::start() {
	cout << "drive start called" << endl;
	runThread = true;
	driveThread = thread(&DriveBaseSubsystem::drive, this);
	shiftThread = thread(&DriveBaseSubsystem::shift, this);
}

void DriveBaseSubsystem::drive() {
	while (!ds->IsEnabled()) {;}
	subsystemHome();

	while(runThread) {
		driveThreadControlStart = Timer::GetFPGATimestamp();

		/*cout << "Left Position: " << leftDrive->GetPosition() << endl;
		cout << "Right Position: " << rightDrive->GetPosition() << endl;
		cout << "Yaw: " << navX->GetYaw() << endl;*/

		if (ds->IsOperatorControl()) {

			x = driveJoystick->GetRawAxis(DRIVE_X_AXIS);
			y = -driveJoystick->GetRawAxis(DRIVE_Y_AXIS);

			x = abs(x) > DRIVE_JOYSTICK_DEADBAND ? x : 0;
			y = abs(y) > DRIVE_JOYSTICK_DEADBAND ? y : 0;

			if (x != 0)
				x = sgn(x) * ((abs(x) - DRIVE_JOYSTICK_DEADBAND) / (1 - DRIVE_JOYSTICK_DEADBAND));
			if (y != 0)
				y = sgn(y) * ((abs(y) - DRIVE_JOYSTICK_DEADBAND) / (1 - DRIVE_JOYSTICK_DEADBAND));

			left = y + x;
			right = (y - x) * -1;
			absLeft = abs(left);
			absRight = abs(right);

			if(absLeft > absRight && absLeft > MAX_OUTPUT) {
				normalLeft = left / absLeft;
				normalRight = right / absLeft;
			}
			else if(absRight > absLeft && absRight > MAX_OUTPUT) {
				normalLeft = left / absRight;
				normalRight = right / absRight;
			}
			else {
				normalLeft = left;
				normalRight = right;
			}

			leftDrive->Set(normalLeft);
			rightDrive->Set(normalRight);

		} else if (ds->IsAutonomous()) {
			if (ds->GetAlliance() == DriverStation::Alliance::kRed) {
				leftDrive->Set(0.75);
				rightDrive->Set(-0.75);

				do {
					avgPosTmp = (abs(leftDrive->GetPosition()) + abs(rightDrive->GetPosition())) / 2;
					this_thread::sleep_for(chrono::milliseconds(10));
				} while (avgPosTmp < STEP_ONE_RED && ds->IsAutonomous());

				leftDrive->Set(0.5);
				rightDrive->Set(0.5);

				avgPosTmp = 0;

				do {
					avgPosTmp = abs(navX->GetYaw());
					this_thread::sleep_for(chrono::milliseconds(10));
				} while (avgPosTmp < TURN_ANGLE_ONE_RED && ds->IsAutonomous());
				leftDrive->Set(0);
				rightDrive->Set(0);
				this_thread::sleep_for(chrono::milliseconds(100));
				leftDrive->SetEncPosition(0);
				rightDrive->SetEncPosition(0);
				this_thread::sleep_for(chrono::milliseconds(30));
				leftDrive->SetEncPosition(0);
				rightDrive->SetEncPosition(0);
				this_thread::sleep_for(chrono::milliseconds(200));
				leftDrive->Set(0.7);
				rightDrive->Set(-0.75);

				avgPosTmp = 0;

				do {
					avgPosTmp = (abs(leftDrive->GetPosition()) + abs(rightDrive->GetPosition())) / 2;
					this_thread::sleep_for(chrono::milliseconds(10));
				} while (avgPosTmp < STEP_TWO && ds->IsAutonomous());

			} else {
				leftDrive->Set(0.75);
				rightDrive->Set(-0.75);

				do {
					avgPosTmp = (abs(leftDrive->GetPosition()) + abs(rightDrive->GetPosition())) / 2;
					this_thread::sleep_for(chrono::milliseconds(10));
				} while (avgPosTmp < STEP_ONE_BLUE && ds->IsAutonomous());

				leftDrive->Set(-0.5);
				rightDrive->Set(-0.5);

				avgPosTmp = 0;

				do {
					avgPosTmp = abs(navX->GetYaw());
					this_thread::sleep_for(chrono::milliseconds(10));
				} while (avgPosTmp < TURN_ANGLE_ONE_BLUE && ds->IsAutonomous());
				leftDrive->Set(0);
				rightDrive->Set(0);
				this_thread::sleep_for(chrono::milliseconds(100));
				leftDrive->SetEncPosition(0);
				rightDrive->SetEncPosition(0);
				this_thread::sleep_for(chrono::milliseconds(30));
				leftDrive->SetEncPosition(0);
				rightDrive->SetEncPosition(0);
				this_thread::sleep_for(chrono::milliseconds(200));
				leftDrive->Set(0.7);
				rightDrive->Set(-0.75);

				avgPosTmp = 0;

				do {
					avgPosTmp = (abs(leftDrive->GetPosition()) + abs(rightDrive->GetPosition())) / 2;
					this_thread::sleep_for(chrono::milliseconds(10));
				} while (avgPosTmp < STEP_TWO && ds->IsAutonomous());

			}
			leftDrive->Set(0);
			rightDrive->Set(0);

			autoDriveFinished = true;

			while (ds->IsAutonomous()) {this_thread::sleep_for(chrono::milliseconds(20));}
		}


		do {
			driveThreadControlEnd = Timer::GetFPGATimestamp();
			driveThreadControlElapsedTimeMS = (int) ((driveThreadControlEnd - driveThreadControlStart) * 1000);
			if (driveThreadControlElapsedTimeMS < MIN_DRIVE_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_DRIVE_LOOP_TIME - driveThreadControlElapsedTimeMS));
		} while(driveThreadControlElapsedTimeMS < MIN_DRIVE_LOOP_TIME);
	}
}

void DriveBaseSubsystem::shift() {
	while (!ds->IsEnabled()) {;}

	while(runThread) {
		shiftThreadControlStart = Timer::GetFPGATimestamp();

		if(driveJoystick->GetRawButton(DRIVE_SHIFT_HIGH)) {
			highGear = true;
			driveGear = DoubleSolenoid::kForward;
		}
		else if(driveJoystick->GetRawButton(DRIVE_SHIFT_LOW)) {
			highGear = false;
			driveGear = DoubleSolenoid::kReverse;
		}

		holdLow = driveJoystick->GetRawButton(DRIVE_SHIFT_HOLDLOW);

		if(holdLow)
			shiftSol->Set(DoubleSolenoid::kReverse);
		else
			shiftSol->Set(driveGear);

		do {
			shiftThreadControlEnd = Timer::GetFPGATimestamp();
			shiftThreadControlElapsedTimeMS = (int) ((shiftThreadControlEnd - shiftThreadControlStart) * 1000);
			if (shiftThreadControlElapsedTimeMS < MIN_SHIFT_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_SHIFT_LOOP_TIME - shiftThreadControlElapsedTimeMS));
		} while(shiftThreadControlElapsedTimeMS < MIN_SHIFT_LOOP_TIME);
	}
}

void DriveBaseSubsystem::subsystemHome() {
	navX->ZeroYaw();
	leftDrive->SetEncPosition(0);
	rightDrive->SetEncPosition(0);
	this_thread::sleep_for(chrono::milliseconds(20));
	leftDrive->SetEncPosition(0);
	rightDrive->SetEncPosition(0);
	this_thread::sleep_for(chrono::milliseconds(200));
}

void DriveBaseSubsystem::stop() {
	cout << "drive stop called" << endl;
	runThread = false;
	if (driveThread.joinable())
		driveThread.join();

	if (shiftThread.joinable())
		shiftThread.join();

	shiftSol->Set(DoubleSolenoid::kReverse);
	driveGear = DoubleSolenoid::kReverse;
	holdLow = false;
	normalLeft = 0;
	normalRight = 0;
	leftDrive->Set(0);
	rightDrive->Set(0);
}

bool DriveBaseSubsystem::isHighGear() {
	return highGear;
}

bool DriveBaseSubsystem::isAutoDriveFinished() {
	return autoDriveFinished;
}

double DriveBaseSubsystem::sgn(double x) {
    return (x > 0) - (x < 0);
}

DriveMotorValues DriveBaseSubsystem::driveAtAngle(double angle, DriveMotorValues values) {
	currentValTmp = angleController->calculatePID(angle, navX->GetYaw());
	if (currentValTmp == 0) {
		return values;
	} else {
		return values;
	}
}
