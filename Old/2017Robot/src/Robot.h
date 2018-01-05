#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "WPILib.h"
#include "Utilities/Controllers.h"
#include "Utilities/CustomSubsystem.h"
#include "Subsystems/DriveBaseSubsystem.h"
#include "Subsystems/GearSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/BallFeederSubsystem.h"
#include "Subsystems/TurretSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/VisionReceiverSubsystem.h"
#include <vector>

#include "AHRS.h"

using namespace frc;

class Robot: public SampleRobot {
public:
	//PWM *leds;
	//DigitalModule *mod;
	Joystick *joystick;
	DigitalOutput *light;
	CameraServer *cs;

	void RobotInit() override;
	void Autonomous() override;
	void OperatorControl() override;
	void Test() override;
private:

	Controllers *robotControllers;
	vector<CustomSubsystem*> subsystemVector;
	DriveBaseSubsystem *robotDrive;
	GearSubsystem *robotGearSubsystem;
	IntakeSubsystem *robotIntake;
	BallFeederSubsystem *robotBallFeeder;
	TurretSubsystem *shooterTurret;
	ShooterSubsystem *ballShooter;
	VisionReceiverSubsystem *visionReceiver;
	AHRS *navX;

	int counter;
};

#endif /* SRC_ROBOT_H_ */
