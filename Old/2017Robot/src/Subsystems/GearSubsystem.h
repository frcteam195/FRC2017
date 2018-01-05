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
#define GEAR_DOWN_ACTION_TIMEOUT_MS 2000

#define MIN_GEAR_LOOP_TIME 10

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
private:
	double gearThreadControlStart, gearThreadControlEnd;
	int gearThreadControlElapsedTimeMS;

	DriverStation *ds;

	Joystick *armJoystick;
	Joystick *driveJoystick;

	CANTalon *gearActuator;

	Solenoid *gearClamp;

	bool runThread;
	thread gearSubsystemThread;

	double timeoutStart, timeoutEnd;
	int timeoutElapsedTimeMS;

	void runGearSubsystem();
};

#endif /* SRC_SUBSYSTEMS_GEARSUBSYSTEM_H_ */
