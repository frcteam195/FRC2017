#ifndef SRC_UTILITIES_CONTROLLERS_H_
#define SRC_UTILITIES_CONTROLLERS_H_

#include "WPILib.h"
#include "CANTalon.h"
#include "AHRS.h"
#include "KnightJoystick.h"

enum AutoSelection {kHopper, kSideGearHopper, kSideGear, kCenterGear, kFarHopper};
enum ShotSelection {hopper, gearFar, gearCenter, loadingZone, removeBalls, launchPadLine, paradeShot};

class Controllers {
private:
	KnightJoystick *driveJoystick;
	KnightJoystick *armJoystick;
	KnightJoystick *buttonBox1;
	KnightJoystick *buttonBox2;

	CANTalon *leftDrive1;
	CANTalon *leftDrive2;
	CANTalon *leftDrive3;
	CANTalon *rightDrive1;
	CANTalon *rightDrive2;
	CANTalon *rightDrive3;
	DoubleSolenoid *shiftSol;

	CANTalon *gearActuator;
	CANTalon *gearRoller;
	Solenoid *gearClamp;
	Solenoid *gearPusher;
	Solenoid *gearFingerSol;
	DigitalInput *gearSwitch;

	CANTalon *intakeMotor;
	CANTalon *intakeMotor2;
	DoubleSolenoid *intakeSol;
	Solenoid *releaseSol;

	CANTalon *turretMotor;
	DigitalInput *turretSwitch;

	CANTalon *shooterWheel;
	CANTalon *hoodMotor;

	CANTalon *carouselMotor;
	CANTalon *feederMotor;

	AHRS *navX;

	ShotSelection selectedShot;

	int driveSubsystemIndex;
	int gearSubsystemIndex;
	int intakeSubsystemIndex;
	int visionSubsystemIndex;
	int turretSubsystemIndex;
	int shooterSubsystemIndex;
	int feederSubsystemIndex;
	int dashboardReporterSubsystem;

public:
	Controllers();
	~Controllers();

	KnightJoystick* getDriveJoystick();
	KnightJoystick* getArmJoystick();
	KnightJoystick* getButtonBox1();
	KnightJoystick* getButtonBox2();

	CANTalon* getLeftDrive1();
	CANTalon* getLeftDrive2();
	CANTalon* getLeftDrive3();
	CANTalon* getRightDrive1();
	CANTalon* getRightDrive2();
	CANTalon* getRightDrive3();
	DoubleSolenoid* getShiftSol();

	CANTalon* getGearActuator();
	CANTalon* getGearRoller();
	Solenoid* getGearClamp();
	Solenoid* getGearPusher();
	Solenoid* getGearFingerSol();
	DigitalInput* getGearSwitch();

	CANTalon* getIntakeMotor();
	CANTalon* getIntakeMotor2();
	DoubleSolenoid* getIntakeSol();
	Solenoid* getReleaseSol();

	CANTalon* getTurretMotor();
	DigitalInput* getTurretSwitch();

	CANTalon* getShooterWheel();
	CANTalon* getHoodMotor();

	CANTalon* getCarouselMotor();
	CANTalon* getFeederMotor();

	AHRS*	getNavX();

	int getDriveSubsystemIndex();
	int getGearSubsystemIndex();
	int getIntakeSubsystemIndex();
	int getVisionSubsystemIndex();
	int getTurretSubsystemIndex();
	int getShooterSubsystemIndex();
	int getFeederSubsystemIndex();
	int getDashboardReporterSubsystemIndex();
	ShotSelection getSelectedShot();

	void setDriveSubsystemIndex(int i);
	void setGearSubsystemIndex(int i);
	void setIntakeSubsystemIndex(int i);
	void setVisionSubsystemIndex(int i);
	void setTurretSubsystemIndex(int i);
	void setShooterSubsystemIndex(int i);
	void setFeederSubsystemIndex(int i);
	void setDashboardReporterSubsystemIndex(int i);
	void setSelectedShot(ShotSelection selectedShot);
};

#endif /* SRC_UTILITIES_CONTROLLERS_H_ */
