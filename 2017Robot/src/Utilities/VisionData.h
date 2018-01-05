#ifndef SRC_UTILITIES_VISIONDATA_H_
#define SRC_UTILITIES_VISIONDATA_H_

struct VisionData {
	double angleDeviation;
	double targetDistance;
	bool onTarget;
	bool targetFound;
	unsigned long sequence;
};

#endif /* SRC_UTILITIES_VISIONDATA_H_ */
