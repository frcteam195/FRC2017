#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "WPILib.h"
#include "Utilities/Controllers.h"
#include "Utilities/CustomSubsystem.h"
#include "Subsystems/DashboardReporterSubsystem.h"
#include "Subsystems/DriveBaseSubsystem.h"
#include "Subsystems/GearSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/BallFeederSubsystem.h"
#include "Subsystems/TurretSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/VisionReceiverSubsystem.h"
#include "Subsystems/HIDControllerSubsystem.h"
#include "Subsystems/AutoSelectionUDPReceiver.h"
#include <vector>

#include <Autonomous/Red/AutoHopperRed.h>
#include <Autonomous/Red/AutoFarHopperRed.h>
#include <Autonomous/Red/AutoRedBoilerSideGear.h>
#include <Autonomous/Red/AutoRedBoilerSideGearHopper.h>
#include <Autonomous/Red/AutoRedCenterGear.h>
#include <Autonomous/Red/AutoHopperRedMotionMagic.h>

#include <Autonomous/Blue/AutoFarHopperBlue.h>
#include <Autonomous/Blue/AutoHopperBlue.h>
#include <Autonomous/Blue/AutoBlueBoilerSideGear.h>
#include <Autonomous/Blue/AutoBlueBoilerSideGearHopper.h>
#include <Autonomous/Blue/AutoBlueCenterGear.h>

#include "AHRS.h"

using namespace frc;

class Robot: public SampleRobot {
public:
	CameraServer *cs;

	void RobotInit() override;
	void Autonomous() override;
	void OperatorControl() override;
	void Test() override;
private:

	AutoSelection selectedAuto;

	AutoFarHopperRed *autoFarHopperRed;
	AutoHopperRed *autoHopperRed;
	//AutoHopperRedMotionMagic *autoHopperRed;
	AutoRedBoilerSideGear *autoRedBoilerSideGear;
	AutoRedBoilerSideGearHopper *autoRedBoilerSideGearHopper;
	AutoRedCenterGear *autoRedCenterGear;

	AutoFarHopperBlue *autoFarHopperBlue;
	AutoHopperBlue *autoHopperBlue;
	AutoBlueBoilerSideGear *autoBlueBoilerSideGear;
	AutoBlueBoilerSideGearHopper *autoBlueBoilerSideGearHopper;
	AutoBlueCenterGear *autoBlueCenterGear;

	Controllers *robotControllers;
	vector<CustomSubsystem*> subsystemVector;
	DriveBaseSubsystem *robotDrive;
	GearSubsystem *robotGearSubsystem;
	IntakeSubsystem *robotIntake;
	BallFeederSubsystem *robotBallFeeder;
	TurretSubsystem *shooterTurret;
	ShooterSubsystem *ballShooter;
	VisionReceiverSubsystem *visionReceiver;
	DashboardReporterSubsystem *dashboardReporterSubsystem;

	HIDControllerSubsystem *hid;
	AutoSelectionUDPReceiver *autoSelector;

	int counter;
};

#endif /* SRC_ROBOT_H_ */
