/*
 * Output.h
 * Output for Jaco2 robot
 *  Created on: 14.07.2016
 *      Author: yitao
 */

#ifndef OUTPUT_H_
#define OUTPUT_H_

#include <vector>
#include "KinovaTypes.h"

class Output{
public:
	Output();
	~Output();

	void Update();

	void  setJointPosition(std::vector<double> var);
	void  setJointVelocity(std::vector<double> var);

	enum OutputType { POSITION, VELOCITY};
	OutputType Type;

private:
	void * commandLayer_handle;
	int (*MySendBasicTrajectory)(TrajectoryPoint command);

	unsigned int nJoint;

	std::vector<bool> OutputSelectionVector;

	std::vector<double> JointPosition;
	std::vector<double> JointVelocity;
};

#endif /* OUTPUT_H_ */
