/*
 * RobotThread.h
 *
 *  Created on: 11.07.2016
 *      Author: yitao
 */

#ifndef ROBOTTHREAD_H_
#define ROBOTTHREAD_H_
#include <thread>
#include <chrono>

class RobotThread{
public:
	RobotThread();
	~RobotThread();
	void 	startThread();
	void	stopThread();

	bool getStatus(){
		return Status;
	};
	void ControlThread();
	volatile bool Activate;
protected:

private:
	volatile bool Status;
	unsigned int FSample;
	std::thread TheThread;

};

class LoopTime{
public:
	LoopTime(unsigned int fs);
	~LoopTime(){}
	void MeasureTime();
	void Delay();
private:
	unsigned int FSample;
	std::chrono::steady_clock::time_point time;
	std::chrono::duration<double, std::micro> time_span;

};

#endif /* ROBOTTHREAD_H_ */
