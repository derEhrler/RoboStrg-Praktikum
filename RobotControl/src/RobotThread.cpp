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

	RobotIO Robot;
	Robot.Init();
	Robot.Home();
	Robot.Update();

	//Set Setpoint
	std::vector<double> setPoint = Robot.getPosition();
	setPoint[0] += 100;
	//Set P-gain
	std::vector<double> pg {12,0,0,0,0,0};

	std::vector<double> ig {19.2,0,0,0,0,0};
	std::vector<double> dg1 = {1.875,0,0,0,0,0};
	std::vector<double> dg2 = {1.875,0,0,0,0,0};

	//Init control value
	std::vector<double> u  {5,0,0,0,0,0};


	// Go to PID.h and PID.cpp and program the controller
	PID PositionController(pg,ig,dg1,dg2,FSample);

	PositionController.setPreviousCurrentValue(Robot.getPosition());
	PositionController.setPreviousSetValue(setPoint);


	LoopTime LT(FSample);

	// Main loop starting here
	int Schritte = 707;
	double Trajektorie [Schritte][6];
	string tmp;
	string P_filename("P_Matrix.txt");
	ifstream input(P_filename);
	if(input)
	{
		for(int i = 0; i < (Schritte); i++)
		{
			for(int j = 0; j < 6; j++) {
				input >> tmp;
				Trajektorie[i][j] = stof(tmp);
			}
		}
	}

	for (int i = 0; i<6;i++) {
		cout << Trajektorie[50][i] << " ";
	}
	cout << endl;


	Store Data;
	Data.open();

	double posError;
	double veloError;
	double stopCondition = 3;
	int counter = 0;
	int trajectorieCnt = 0;
	while(Activate){
		//Data.write(to_string((Robot.getPosition()[0])));

		for (int j = 0; j < 6; j++) {
			setPoint[j] = Trajektorie[trajectorieCnt][j];

		u = PositionController.calculate(setPoint,Robot.getPosition());

		Robot.setVelocity(u);
		Robot.Update();

		// Delays the loop so that Fsample is fulfilled
		LT.Delay();
		}
		//STOP function
		posError = setPoint[0]-Robot.getPosition()[0];

		if(posError < 0)
			posError *= -1;


		if(posError < stopCondition)
		{
			counter ++;
		}
		else {
			counter = 0;
		}
		cout << counter << endl;

		if(counter >= 50)
			Activate = 0;

		trajectorieCnt++;
		if (trajectorieCnt >= Schritte)
			Activate = !Activate;

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
