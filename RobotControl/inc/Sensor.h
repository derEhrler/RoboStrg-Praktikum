/*
 * Sensor.h
 *
 *  Created on: 11.07.2016
 *      Author: yitao
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <vector>
#include "KinovaTypes.h"

class Sensor{
public:
	Sensor();
	~Sensor();

	void Update();

	std::vector<double>  getJointPosition();
	std::vector<double>  getJointVelocity();
	std::vector<double>  getJointForceTorque();


private:
	void * commandLayer_handle;
	int (*MyGetAngularPosition)(AngularPosition &);
	int (*MyGetAngularVelocity)(AngularPosition &);
	int (*MyGetAngularForce)(AngularPosition &);


	unsigned int nJointPosition;
	unsigned int nJointVelocity;
	unsigned int nJointForceTorque;

	std::vector<bool> SensorSelectionVector;

	std::vector<double> JointPosition;

	std::vector<double> JointVelocity;
	std::vector<double> JointForceTorque;
};

#endif /* SENSOR_H_ */
