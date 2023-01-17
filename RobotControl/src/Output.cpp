/*
 * Output.cpp
 *
 *  Created on: 14.07.2016
 *      Author: yitao
 */




#include "Output.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include <dlfcn.h>
#include <algorithm>
#include <iostream>


Output::Output(){

	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");

	if (MySendBasicTrajectory == NULL){
		std::cout << "Error: Output not initiated!" << std::endl;
	}

	nJoint = 6;
	Type = VELOCITY;

	for(unsigned int i = 0; i < nJoint ; i++){
		JointPosition.push_back(0);
		JointVelocity.push_back(0);
	}
}

Output::~Output(){

}

void Output::Update(){

	TrajectoryPoint joint_data;

	if (Type == VELOCITY){
		joint_data.Position.Type = ANGULAR_VELOCITY;

		joint_data.Position.Actuators.Actuator1 = JointVelocity[0];
		joint_data.Position.Actuators.Actuator2 = JointVelocity[1];
		joint_data.Position.Actuators.Actuator3 = JointVelocity[2];
		joint_data.Position.Actuators.Actuator4 = JointVelocity[3];
		joint_data.Position.Actuators.Actuator5 = JointVelocity[4];
		joint_data.Position.Actuators.Actuator6 = JointVelocity[5];

		// Fingers don't have velocity control
		joint_data.Position.Fingers.Finger1 = 0;
		joint_data.Position.Fingers.Finger2 = 0;
		joint_data.Position.Fingers.Finger3 = 0;
	}
	else if(Type == POSITION ){

		joint_data.Position.Type = ANGULAR_POSITION;

		joint_data.Position.Actuators.Actuator1 = JointPosition[0];
		joint_data.Position.Actuators.Actuator2 = JointPosition[1];
		joint_data.Position.Actuators.Actuator3 = JointPosition[2];
		joint_data.Position.Actuators.Actuator4 = JointPosition[3];
		joint_data.Position.Actuators.Actuator5 = JointPosition[4];
		joint_data.Position.Actuators.Actuator6 = JointPosition[5];

		// No support for fingers yet
		joint_data.Position.Fingers.Finger1 = 0;
		joint_data.Position.Fingers.Finger2 = 0;
		joint_data.Position.Fingers.Finger3 = 0;
	}

	MySendBasicTrajectory(joint_data);
}

void Output::setJointPosition(std::vector<double> var){
	JointPosition = var;
}
void Output::setJointVelocity(std::vector<double> var){
	JointVelocity = var;
}
