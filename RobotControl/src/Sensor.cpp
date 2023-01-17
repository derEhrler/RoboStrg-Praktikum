/*
 * Sensor.cpp
 *
 *  Created on: 11.07.2016
 *      Author: yitao
 */

#include "Sensor.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <vector>
#include <dlfcn.h>
#include <algorithm>
#include <iostream>

Sensor::Sensor(){

	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
	MyGetAngularVelocity = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularVelocity");
	MyGetAngularForce = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularForce");

	if (MyGetAngularPosition == NULL || MyGetAngularVelocity == NULL || MyGetAngularForce == NULL){
		std::cout << "Error: Sensors not initiated!" << std::endl;
	}

	nJointPosition = 6;
	nJointVelocity = 6;
	nJointForceTorque = 6;

	for(unsigned int i = 0; i < nJointPosition ; i++){
		JointPosition.push_back(0);
		JointVelocity.push_back(0);
		JointForceTorque.push_back(0);
	}
}

Sensor::~Sensor(){

}

void Sensor::Update(){

	AngularPosition joint_data;
	MyGetAngularPosition(joint_data);

	JointPosition[0] = joint_data.Actuators.Actuator1;
	JointPosition[1] = joint_data.Actuators.Actuator2;
	JointPosition[2] = joint_data.Actuators.Actuator3;
	JointPosition[3] = joint_data.Actuators.Actuator4;
	JointPosition[4] = joint_data.Actuators.Actuator5;
	JointPosition[5] = joint_data.Actuators.Actuator6;

	/* Better calculate joint velocity from position
	MyGetAngularVelocity(joint_data);

	JointVelocity[0] = joint_data.Actuators.Actuator1;
	JointVelocity[1] = joint_data.Actuators.Actuator2;
	JointVelocity[2] = joint_data.Actuators.Actuator3;
	JointVelocity[3] = joint_data.Actuators.Actuator4;
	JointVelocity[4] = joint_data.Actuators.Actuator5;
	JointVelocity[5] = joint_data.Actuators.Actuator6;
	*/

	MyGetAngularVelocity(joint_data);

	JointForceTorque[0] = joint_data.Actuators.Actuator1;
	JointForceTorque[1] = joint_data.Actuators.Actuator2;
	JointForceTorque[2] = joint_data.Actuators.Actuator3;
	JointForceTorque[3] = joint_data.Actuators.Actuator4;
	JointForceTorque[4] = joint_data.Actuators.Actuator5;
	JointForceTorque[5] = joint_data.Actuators.Actuator6;
}

std::vector<double>  Sensor::getJointPosition(){
	return JointPosition;
}
std::vector<double>  Sensor::getJointVelocity(){
	return JointVelocity;
}
std::vector<double>  Sensor::getJointForceTorque(){
	return JointForceTorque;
}
