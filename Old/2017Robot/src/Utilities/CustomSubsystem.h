#ifndef SRC_SUBSYSTEMS_CUSTOMSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_CUSTOMSUBSYSTEM_H_

class CustomSubsystem {
public:
	virtual ~CustomSubsystem() {};
	virtual void init() = 0;
	virtual void start() = 0;
	virtual void subsystemHome() = 0;
	virtual void stop() = 0;
};

#endif /* SRC_SUBSYSTEMS_CUSTOMSUBSYSTEM_H_ */
