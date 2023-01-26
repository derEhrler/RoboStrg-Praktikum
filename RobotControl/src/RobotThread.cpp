/*
 * RobotThread.cpp
 *
 *  Created on: 11.07.2016
 *      Author: yitao
 */

#include "RobotThread.h"
#include "PID.h"
#include <iostream>
#include "RobotIO.h"
#include "Store.h"
#include <fstream>
#include <string>
#include "Drehmoment.h"
using namespace std;

RobotThread::RobotThread():TheThread(){
	Status = 0;
	Activate = 0;
	FSample = 100;
}

RobotThread::~RobotThread(){
	RobotThread::stopThread();
}

void RobotThread::startThread(){
	if (Status == 0){
		std::cout << "Starting robot thread" << std::endl;
		Activate = 1;
		TheThread = std::thread(&RobotThread::ControlThread,this);
	}
	else{
		std::cout << "Thread already running" << std::endl;
	}
}

void RobotThread::stopThread(){

	if(TheThread.joinable()){
		std::cout << "Stopping Thread ..." << std::endl;
		Activate = 0;
		TheThread.join();
		std::cout << "Thread stopped" << std::endl;
	}

}

void RobotThread::ControlThread(){
	std::cout << "Control-thread started" << std::endl;
	Status = 1; // Set Thread to Active

	// init robot
	RobotIO Robot;
	Robot.Init();
	Robot.Home();
	Robot.Update();

	//Set Setpoint
	std::vector<double> setPoint = Robot.getPosition();
	setPoint[0] += 100;

	//Set PID gains
	std::vector<double> pg {12,12,12,12,12,12};
	std::vector<double> ig {19.2,19.2,19.2,19.2,19.2,19.2};
	std::vector<double> dg1 = {1.875,1.875,1.875,1.875,1.875,1.875};
	std::vector<double> dg2 = {1.875,1.875,1.875,1.875,1.875,1.875};

	//Init control value
	std::vector<double> u  {5,0,0,0,0,0};

	// Go to PID.h and PID.cpp and program the controller
	PID PositionController(pg,ig,dg1,dg2,FSample);

	PositionController.setPreviousCurrentValue(Robot.getPosition());
	PositionController.setPreviousSetValue(setPoint);

	LoopTime LT(FSample);

	// read in MATLAB trajektory
	int IKsteps = 707; // length of P_Matrix.txt file
	int trajectorieCnt = 0; // counter for steps taken in IK control loop
	double trajectory [IKsteps][6];
	string tmp;
	string P_filename("P_Matrix_707x6.txt");
	ifstream input(P_filename);

	if(input) {
		for(int i = 0; i < IKsteps; i++) {
			for(int j = 0; j < 6; j++) {
				input >> tmp;
				trajectory[i][j] = stof(tmp);
			}
		}
	}

	// store angles for PID control optimization
	Store Data;
	Data.open();

	// values for main loop
	// Impedanzregelung
	std::vector<double> posError;
	std::vector<double> acceleration {0,0,0,0,0,0};
	std::vector<double> Fsoll {10,10,10,10,10,10};
	std::vector<double> setForce {0,0,0,0,0,0};
	// Stop Function
	double veloError;
	double stopCondition = 3;
	int counter = 0;

	// Main loop starting here
	while(Activate){

		// write angles to file for PID control optimization
		Data.write(to_string((Robot.getPosition()[0])));

		// update setpoint with next trajectory value
		for (int j = 0; j < 6; j++) {
			setPoint[j] = trajectory[trajectorieCnt][j];
		}

		// calculate u und update robot
		posError = setPoint-Robot.getPosition(); // delta x = x_soll - Positionsmessung
		Drehmoment(Robot.getPosition(),Robot.JointVelocity,acceleration,Robot.GetTorque());



		u = PositionController.calculate(setForce,Robot.GetTorque()); // V3 Impedanzregelung
		//u = PositionController.calculate(setPoint,Robot.getPosition()); // V2 Positionsregelung

		Robot.setVelocity(u);
		Robot.Update();

		// Delays the loop so that Fsample is fulfilled
		LT.Delay();

		//STOP function
		/*
		posError = setPoint[0]-Robot.getPosition()[0];

		if(posError < 0)
			posError *= -1;

		if(posError < stopCondition)
			counter ++;
		else
			counter = 0;

		if(counter >= 2000)
			Activate = 0;
		*/

		// iterate to next point and stop if eof is reached
		trajectorieCnt++;
		if (trajectorieCnt >= IKsteps)
			Activate = !Activate;

		//std::vector<double> tmp = Robot.getPosition();
		//cout << tmp[0] << " " << tmp[1] << " " << tmp[2]<< " " << tmp[3]<< " " << tmp[4]<< " " << tmp[5] << endl;
	}

	Status = 0;
	std::cout << "End of thread reached: " << std::endl;
}


LoopTime::LoopTime(unsigned int fs){
	FSample = fs;
	LoopTime::MeasureTime();
}

void LoopTime::MeasureTime(){
	time = std::chrono::steady_clock::now();
}

void LoopTime::Delay(){
	time_span = (std::chrono::steady_clock::now() - time);
	if ((std::chrono::microseconds(1000000/FSample) - time_span)>= std::chrono::microseconds(0)){
		std::this_thread::sleep_for(std::chrono::microseconds(1000000/FSample) - time_span);
		time = std::chrono::steady_clock::now();
	}
	else{
		//std::cout << "System Load too high!" << std::endl;
		time = std::chrono::steady_clock::now();
	}
}
