#include "Controllers.h"

using namespace frc;

Controllers::Controllers() {
	//Drive Joystick Setup
	driveJoystick = new Joystick(0);
	armJoystick = new Joystick(1);
	buttonBox1 = new Joystick(2);
	buttonBox2 = new Joystick(3);


	//Left Drive Setup
	leftDrive1 = new CANTalon(1);
	leftDrive2 = new CANTalon(2);
	leftDrive3 = new CANTalon(3);

	//Right Drive Setup
	rightDrive1 = new CANTalon(4);
	rightDrive2 = new CANTalon(5);
	rightDrive3 = new CANTalon(6);

	//Shift Solenoid Setup
	shiftSol = new DoubleSolenoid(0, 1);

	//Gear Setup
	gearActuator = new CANTalon(14);
	gearClamp = new Solenoid(6);

	//Turret Setup
	turretMotor = new CANTalon(9, 1);

	//Shooter Setup
	shooterWheel = new CANTalon(10);
	hoodMotor = new CANTalon(13);

	//Intake Setup
	intakeMotor = new CANTalon(12);
	intakeMotor2 = new CANTalon(11);
	intakeSol = new DoubleSolenoid(2, 3);
	releaseSol = new DoubleSolenoid(4, 5);

	//Ball Feeder Setup
	carouselMotor = new CANTalon(7);
	feederMotor = new CANTalon(8);

	selectedShot = ShotSelection::hopper;

	driveSubsystemIndex = -1;
	gearSubsystemIndex = -1;
	intakeSubsystemIndex = -1;
	visionSubsystemIndex = -1;
	turretSubsystemIndex = -1;
	shooterSubsystemIndex = -1;
	feederSubsystemIndex = -1;

    try {
        navX = new AHRS(SPI::Port::kMXP);
    	//navX = 0;
    } catch (std::exception& ex ) {
        std::string err_string = "Error instantiating navX MXP:  ";
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }
}

Joystick* Controllers::getDriveJoystick() {
	return driveJoystick;
}

Joystick* Controllers::getArmJoystick() {
	return armJoystick;
}

Joystick* Controllers::getButtonBox1() {
	return buttonBox1;
}

Joystick* Controllers::getButtonBox2() {
	return buttonBox2;
}

CANTalon* Controllers::getLeftDrive1() {
	return leftDrive1;
}

CANTalon* Controllers::getLeftDrive2() {
	return leftDrive2;
}
CANTalon* Controllers::getLeftDrive3() {
	return leftDrive3;
}

CANTalon* Controllers::getRightDrive1() {
	return rightDrive1;
}

CANTalon* Controllers::getRightDrive2() {
	return rightDrive2;
}

CANTalon* Controllers::getRightDrive3() {
	return rightDrive3;
}

DoubleSolenoid* Controllers::getShiftSol() {
	return shiftSol;
}

CANTalon* Controllers::getGearActuator() {
	return gearActuator;
}

Solenoid* Controllers::getGearClamp() {
	return gearClamp;
}

CANTalon* Controllers::getIntakeMotor() {
	return intakeMotor;
}

CANTalon* Controllers::getIntakeMotor2() {
	return intakeMotor2;
}

DoubleSolenoid* Controllers::getIntakeSol() {
	return intakeSol;
}

DoubleSolenoid* Controllers::getReleaseSol() {
	return releaseSol;
}

CANTalon* Controllers::getTurretMotor() {
	return turretMotor;
}

CANTalon* Controllers::getShooterWheel() {
	return shooterWheel;
}

CANTalon* Controllers::getHoodMotor() {
	return hoodMotor;
}

CANTalon* Controllers::getCarouselMotor() {
	return carouselMotor;
}

CANTalon* Controllers::getFeederMotor() {
	return feederMotor;
}

AHRS* Controllers::getNavX() {
	return navX;
}

int Controllers::getDriveSubsystemIndex() {
	return driveSubsystemIndex;
}

int Controllers::getGearSubsystemIndex() {
	return gearSubsystemIndex;
}

int Controllers::getIntakeSubsystemIndex() {
	return intakeSubsystemIndex;
}

int Controllers::getVisionSubsystemIndex() {
	return visionSubsystemIndex;
}

int Controllers::getTurretSubsystemIndex() {
	return turretSubsystemIndex;
}

int Controllers::getShooterSubsystemIndex() {
	return shooterSubsystemIndex;
}

int Controllers::getFeederSubsystemIndex() {
	return feederSubsystemIndex;
}

ShotSelection Controllers::getSelectedShot() {
	return selectedShot;
}

void Controllers::setDriveSubsystemIndex(int i) {
	driveSubsystemIndex = i;
}

void Controllers::setGearSubsystemIndex(int i) {
	gearSubsystemIndex = i;
}

void Controllers::setIntakeSubsystemIndex(int i) {
	intakeSubsystemIndex = i;
}

void Controllers::setVisionSubsystemIndex(int i) {
	visionSubsystemIndex = i;
}

void Controllers::setTurretSubsystemIndex(int i) {
	turretSubsystemIndex = i;
}

void Controllers::setShooterSubsystemIndex(int i) {
	shooterSubsystemIndex = i;
}

void Controllers::setFeederSubsystemIndex(int i) {
	feederSubsystemIndex = i;
}

void Controllers::setSelectedShot(ShotSelection selectedShot) {
	this->selectedShot = selectedShot;
}
