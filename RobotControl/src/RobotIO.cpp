/*
 * RobotIO.cpp
 *
 *  Created on: 28.07.2016
 *      Author: yitao
 */
#include "RobotIO.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include <dlfcn.h>
#include <iostream>
#include <unistd.h>

RobotIO::RobotIO(){
	commLayer_Handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);
	InitAPI = (int (*)()) dlsym(commLayer_Handle,"InitAPI");
	CloseAPI = (int (*)()) dlsym(commLayer_Handle,"CloseAPI");
	InitFingers = (int (*)()) dlsym(commLayer_Handle,"InitFingers");
	MoveHome = (int (*)()) dlsym(commLayer_Handle,"MoveHome");
	GetAngularPosition = (int (*)(AngularPosition &)) dlsym(commLayer_Handle,"GetAngularPosition");
	GetTorque = (int (*)(AngularPosition &)) dlsym(commLayer_Handle,"GetAngularForce");
	SendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commLayer_Handle,"SendBasicTrajectory");

	for(unsigned int i = 0; i<6; i++){
		SensorPosition.push_back(0);
		SensorTorque.push_back(0);
		JointVelocity.push_back(0);
	}
}

RobotIO::~RobotIO(){
	Close();
}

int RobotIO::Init(){
	int (*GetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
	GetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commLayer_Handle,"GetDevices");

	int (*SetActiveDevice)(KinovaDevice device);
	SetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commLayer_Handle,"SetActiveDevice");

	if((InitAPI == NULL) || (CloseAPI == NULL) || (SendBasicTrajectory == NULL)	|| (MoveHome == NULL) || (InitFingers == NULL)){
		std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << std::endl;
	}
	else{
		std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << std::endl << std::endl;

		int result = (*InitAPI)();
		std::cout << "Initialization's result :" << result << std::endl;
		KinovaDevice list[MAX_KINOVA_DEVICE];
		int devicesCount = GetDevices(list, result);

		for(int i = 0; i < devicesCount; i++){
			std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" <<std::endl;
			//Setting the current device as the active device.
			SetActiveDevice(list[i]);
		}
		std::cout << "Joints Initialized!" << std::endl;
		return 1;
	}
	return 0;
}

void RobotIO::Close(){
	CloseAPI();
}

void RobotIO::Home(){
	MoveHome();
}

int RobotIO::Update(){
	TrajectoryPoint pointToSend;
	pointToSend.InitStruct();
	pointToSend.Position.Type = ANGULAR_VELOCITY;

	pointToSend.Position.Actuators.Actuator1 = JointVelocity[0];
	pointToSend.Position.Actuators.Actuator2 = JointVelocity[1];
	pointToSend.Position.Actuators.Actuator3 = JointVelocity[2];
	pointToSend.Position.Actuators.Actuator4 = JointVelocity[3];
	pointToSend.Position.Actuators.Actuator5 = JointVelocity[4];
	pointToSend.Position.Actuators.Actuator6 = JointVelocity[5];

	pointToSend.Position.Fingers.Finger1 = 0;
	pointToSend.Position.Fingers.Finger2 = 0;
	pointToSend.Position.Fingers.Finger3 = 0;

	SendBasicTrajectory(pointToSend);

	AngularPosition jointPosition;
	(*GetAngularPosition)(jointPosition);
	SensorPosition[0] = jointPosition.Actuators.Actuator1;
	SensorPosition[1] = jointPosition.Actuators.Actuator2;
	SensorPosition[2] = jointPosition.Actuators.Actuator3;
	SensorPosition[3] = jointPosition.Actuators.Actuator4;
	SensorPosition[4] = jointPosition.Actuators.Actuator5;
	SensorPosition[5] = jointPosition.Actuators.Actuator6;

	(*GetTorque)(jointPosition);
	SensorTorque[0] = jointPosition.Actuators.Actuator1;
	SensorTorque[1] = jointPosition.Actuators.Actuator2;
	SensorTorque[2] = jointPosition.Actuators.Actuator3;
	SensorTorque[3] = jointPosition.Actuators.Actuator4;
	SensorTorque[4] = jointPosition.Actuators.Actuator5;
	SensorTorque[5] = jointPosition.Actuators.Actuator6;

	return 1;
}

void RobotIO::setVelocity(std::vector<double> vel){
	JointVelocity = vel;;
}

std::vector<double> RobotIO::getPosition(){
	return SensorPosition;
}

std::vector<double> RobotIO::getTorque(){
	return SensorTorque;
}
