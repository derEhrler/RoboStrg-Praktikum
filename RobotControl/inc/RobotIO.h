/*
 * RobotIO.h
 *
 *  Created on: 28.07.2016
 *      Author: yitao
 */

#ifndef ROBOTIO_H_
#define ROBOTIO_H_
#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <vector>

class RobotIO{
public:
	RobotIO();
	~RobotIO();

	int Init();
	void Close();
	void Home();
	int Update();

	void setVelocity(std::vector<double> vel);

	std::vector<double> getPosition();
	std::vector<double> getTorque();

protected:
	std::vector<double> SensorPosition;
	std::vector<double> SensorTorque;

	std::vector<double> JointVelocity;

	void *commLayer_Handle;

	int(*InitAPI)();
	int(*CloseAPI)();
	int(*InitFingers)();
	int (*MoveHome)();
	int(*GetAngularPosition)(AngularPosition &);
	int(*GetTorque)(AngularPosition &);
	int(*SendBasicTrajectory)(TrajectoryPoint command);

};



#endif /* ROBOTIO_H_ */
