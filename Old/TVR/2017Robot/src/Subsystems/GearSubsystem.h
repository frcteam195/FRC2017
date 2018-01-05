#ifndef SRC_SUBSYSTEMS_GEARSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_GEARSUBSYSTEM_H_

#include "Utilities/CustomSubsystem.h"
#include "Utilities/Controllers.h"
#include "Utilities/GlobalDefines.h"
#include "Utilities/TuneablePID.h"
#include "WPILib.h"
#include "CANTalon.h"
#include <vector>
#include <thread>

#define POSITION_DOWN -0.220
#define POSITION_ZERO -0.01
#define POSITION_HOLD -0.01
#define POSITION_RELEASE -0.095
//#define POSITION_UP .017334	//Absolute max
#define POSITION_UP .0172
#define DEVIATION_THRESHOLD 0.015
#define GEAR_DOWN_ACTION_TIMEOUT_MS 1200

#define MIN_GEAR_LOOP_TIME 15

using namespace std;
using namespace frc;

class GearSubsystem: public CustomSubsystem, public TuneablePID {
public:
	GearSubsystem(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector);
	~GearSubsystem();

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;
	double getGearActuatorPos();
	void setGearActuatorPos(double pos);
	void setGearClamp(bool open);
	void setGearPusher(bool pushed);
	void configVelocityAccel(double velocity, double accel);

private:
	double gearThreadControlStart, gearThreadControlEnd;
	int gearThreadControlElapsedTimeMS;

	DriverStation *ds;

	CANTalon *gearActuator;

	double gearActuatorPos;
	bool gearClampOpen;
	bool gearPushed;

	Solenoid *gearClamp;
	Solenoid *gearPusher;

	DigitalInput *gearSwitch;

	bool runThread;
	bool homing;
	thread gearSubsystemThread;

	mutex _gearClampMutex;
	mutex _gearPusherMutex;

	double prevVelocity;
	double prevAccel;

	void runGearSubsystem();
};

#endif /* SRC_SUBSYSTEMS_GEARSUBSYSTEM_H_ */
