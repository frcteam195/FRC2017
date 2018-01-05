/*
 * VisionTurretControlAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_VISIONTURRETCONTROLACTION_H_
#define SRC_ACTIONS_VISIONTURRETCONTROLACTION_H_

#include <Utilities/CustomAction.h>
#include <Subsystems/DashboardReporterSubsystem.h>
#include <Subsystems/TurretSubsystem.h>
#include <Subsystems/ShooterSubsystem.h>
#include <Subsystems/VisionReceiverSubsystem.h>

//#define TURRET_VISION_ACCEL 9000
#define TURRET_VISION_ACCEL 2800
#define TURRET_VISION_VEL 1200
#define ON_TARGET_ANGLE_DEVIATION_DEG 1

#define VISION_TURRET_CONTROL_LOOP_TIME_MS 10

class VisionTurretControlAction: public CustomAction {
public:
	VisionTurretControlAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector)
	:CustomAction(robotControllers, subsystemVector){
		turretSubsystem = dynamic_cast<TurretSubsystem*>(subsystemVector->at(robotControllers->getTurretSubsystemIndex()));
		visionReceiverSubsystem = dynamic_cast<VisionReceiverSubsystem*>(subsystemVector->at(robotControllers->getVisionSubsystemIndex()));
		shooterSubsystem = dynamic_cast<ShooterSubsystem*>(subsystemVector->at(robotControllers->getShooterSubsystemIndex()));
		dashboardReporterSubsystem = dynamic_cast<DashboardReporterSubsystem*>(subsystemVector->at(robotControllers->getDashboardReporterSubsystemIndex()));

		prevSequence = -1;
		deviation = 0;
		visionData = nullptr;

		runThread = false;

		targetAvgDistance = 0;
		targetDistanceSum = 0;
		avgDistanceCounter = 0;

		autoRanging = false;

		manualAdjust = false;
	}
	~VisionTurretControlAction() {}

	void start() override {
		if (_actionMutex.try_lock()) {
			_threadSentinelMutex.lock();
			runThread = true;
			_threadSentinelMutex.unlock();
			running = true;
			autoRanging = false;
			runAction = thread(&VisionTurretControlAction::run, this);
		}
	};

	void start(bool autoRanging)  {
		start();
		_threadSentinelMutex.lock();
		this->autoRanging = autoRanging;
		_threadSentinelMutex.unlock();
	};

	void setManualAdjust(bool manualAdjust) {
		_threadSentinelMutex.lock();
		this->manualAdjust = manualAdjust;
		_threadSentinelMutex.unlock();
	}

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
		dashboardReporterSubsystem->setVisionEnabled(false);
	};

	void clearBuffer() {
		_threadSentinelMutex.lock();
		for (int i = 0; i < 10; i++) {
			visionData = visionReceiverSubsystem->getVisionData();
		}
		_threadSentinelMutex.unlock();
	}

	bool isOnTarget() {
		return (deviation / (124.0 / 18.0 / 360.0)) < ON_TARGET_ANGLE_DEVIATION_DEG;
	}

protected:
	void run() override {
		visionReceiverSubsystem->setEnableVision(true);
		dashboardReporterSubsystem->setVisionEnabled(true);
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
	ShooterSubsystem* shooterSubsystem;
	TurretSubsystem *turretSubsystem;
	VisionReceiverSubsystem *visionReceiverSubsystem;
	DashboardReporterSubsystem *dashboardReporterSubsystem;

	VisionData *visionData;
	unsigned long prevSequence;
	double deviation;
	bool runThread;

	bool manualAdjust;

	double targetAvgDistance;
	double targetDistanceSum;
	bool autoRanging;
	int avgDistanceCounter;

	mutex _threadSentinelMutex;

	void processVision() {
		visionData = visionReceiverSubsystem->getVisionData();

		dashboardReporterSubsystem->setOnTarget(visionData->onTarget);
		dashboardReporterSubsystem->setJetsonOperational(visionData->sequence > prevSequence);

		if(!visionData->onTarget && !turretSubsystem->hasError()) {
			if (visionData->targetFound && visionData->sequence > prevSequence) {
				turretSubsystem->configVelocityAccel(TURRET_VISION_VEL, TURRET_VISION_ACCEL);
				deviation = visionData->angleDeviation * 124 / 18 / 360;
				if(turretSubsystem->getTurretPos() + deviation > TURRET_LOWER_SOFT_STOP && turretSubsystem->getTurretPos() + deviation < TURRET_UPPER_SOFT_STOP)
					turretSubsystem->setTurretPos(turretSubsystem->getTurretPos() + deviation);
			}
			else {
				//TODO: Add code to predict boiler position when target is not found
			}

		} else {
			if (++avgDistanceCounter > 0) {
				targetDistanceSum += visionData->targetDistance;
				targetAvgDistance = targetDistanceSum / avgDistanceCounter;
			}

			if (autoRanging && !robotControllers->getArmJoystick()->GetRawButton(SHOOT_BUTTON)) {
				if (!manualAdjust) {
					if (targetAvgDistance <= 135) {
						shooterSubsystem->setHoodPos(0.178);
						shooterSubsystem->setSpeed(-3097 + (-3097 * (targetAvgDistance/135)));
					} else if (targetAvgDistance > 135 && targetAvgDistance <= 143) {
						shooterSubsystem->setHoodPos(0.278);
						shooterSubsystem->setSpeed(-3222 * (targetAvgDistance / ((143+135)/2)));
					} else if (targetAvgDistance > 143 && targetAvgDistance <= 150) {
						shooterSubsystem->setHoodPos(0.328);
						shooterSubsystem->setSpeed(-3322 * (targetAvgDistance / ((150+143)/2)));
					} else if (targetAvgDistance > 150 && targetAvgDistance <= 158) {
						shooterSubsystem->setHoodPos(0.428);
						shooterSubsystem->setSpeed(-3347 * (targetAvgDistance / ((158+150)/2)));
					} else if (targetAvgDistance > 158 && targetAvgDistance <= 165) {
						shooterSubsystem->setHoodPos(0.366);
						shooterSubsystem->setSpeed(-3500 * (targetAvgDistance / ((165+158)/2)));
					} else if (targetAvgDistance > 165) {
						shooterSubsystem->setHoodPos(0.366);
						shooterSubsystem->setSpeed(-3700);
					}
				}

			}

			/*
			if (avgDistanceCounter >= 50) {
				cout << "Target Distance: " << targetAvgDistance << endl;
				cout << "Wheel Speed: " << shooterSubsystem->getSetpoint() << endl;
				cout << "Hood Pos:: " << shooterSubsystem->getHoodSetpoint() << endl;

				cout << endl << endl << endl;

				targetAvgDistance = 0;
				targetDistanceSum = 0;
				avgDistanceCounter = 0;
			}
			*/
		}

		prevSequence =  visionData->sequence;
	}

};

#endif /* SRC_ACTIONS_VISIONTURRETCONTROLACTION_H_ */
