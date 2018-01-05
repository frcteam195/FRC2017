/*
 * VisionTurretControlAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_VISIONTURRETCONTROLACTION_H_
#define SRC_ACTIONS_VISIONTURRETCONTROLACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/TurretSubsystem.h>
#include <Subsystems/VisionReceiverSubsystem.h>

#define TURRET_VISION_ACCEL 9000
#define TURRET_VISION_VEL 1200
#define ON_TARGET_ANGLE_DEVIATION_DEG 1

#define VISION_TURRET_CONTROL_LOOP_TIME_MS 10

class VisionTurretControlAction: public CustomAction {
public:
	VisionTurretControlAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));
		visionReceiverSubsystem = dynamic_cast<VisionReceiverSubsystem*>(subsystemVector->at(robotControllers->getVisionSubsystemIndex()));

		prevSequence = -1;
		deviation = 0;
		visionData = nullptr;

		runThread = false;
	}
	~VisionTurretControlAction() {}

	void start() override {
		if (_actionMutex.try_lock()) {
			_threadSentinelMutex.lock();
			runThread = true;
			_threadSentinelMutex.unlock();
			running = true;
			runAction = thread(&VisionTurretControlAction::run, this);
		}
	};

	void holdStart() {
		if (_actionMutex.try_lock()) {
			_threadSentinelMutex.lock();
			runThread = true;
			_threadSentinelMutex.unlock();
			running = true;
			runAction = thread(&VisionTurretControlAction::run, this);
			while (robotControllers->getArmJoystick()->GetRawButton(VISION_HOLD_ON)) {
				this_thread::sleep_for(chrono::milliseconds(25));
			}
			_threadSentinelMutex.lock();
			runThread = false;
			_threadSentinelMutex.unlock();
		}
	}

	void stop() override {
		_threadSentinelMutex.lock();
		runThread = false;
		_threadSentinelMutex.unlock();
	};

	bool isOnTarget() {
		return (deviation / (124 / 18 / 360)) < ON_TARGET_ANGLE_DEVIATION_DEG;
	}

protected:
	void run() override {
		visionReceiverSubsystem->setEnableVision(true);
		while (runThread) {
			timeoutStart = Timer::GetFPGATimestamp();
			processVision();
			do {
				timeoutEnd = Timer::GetFPGATimestamp();
				timeoutElapsedTimeMS = (int) ((timeoutEnd - timeoutStart) * 1000);
				if (timeoutElapsedTimeMS < VISION_TURRET_CONTROL_LOOP_TIME_MS)
					this_thread::sleep_for(chrono::milliseconds(VISION_TURRET_CONTROL_LOOP_TIME_MS - timeoutElapsedTimeMS));
			} while(timeoutElapsedTimeMS < VISION_TURRET_CONTROL_LOOP_TIME_MS);
		}
		visionReceiverSubsystem->setEnableVision(false);
		running = false;
		runAction.detach();
		_actionMutex.unlock();
	};

private:
	TurretSubsystem *turretSubsystem;
	VisionReceiverSubsystem *visionReceiverSubsystem;

	VisionData *visionData;
	unsigned long prevSequence;
	double deviation;
	bool runThread;

	mutex _threadSentinelMutex;

	void processVision() {
		visionData = visionReceiverSubsystem->getVisionData();
		turretSubsystem->configVelocityAccel(TURRET_VISION_VEL, TURRET_VISION_ACCEL);
		if(!visionData->onTarget) {
			if (visionData->targetFound && visionData->sequence > prevSequence) {
				deviation = visionData->angleDeviation * 124 / 18 / 360;
				if(turretSubsystem->getTurretPos() + deviation > TURRET_LOWER_SOFT_STOP && turretSubsystem->getTurretPos() + deviation < TURRET_UPPER_SOFT_STOP)
					turretSubsystem->setTurretPos(turretSubsystem->getTurretPos() + deviation);
			}
			else {
				//TODO: Add code to predict boiler position when target is not found
			}
		}
		prevSequence =  visionData->sequence;
	}
};

#endif /* SRC_ACTIONS_VISIONTURRETCONTROLACTION_H_ */
