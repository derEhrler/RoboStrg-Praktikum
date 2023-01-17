//============================================================================
// Name        : RobotControl.cpp
// Author      : Yitao
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <chrono>
#include "RobotThread.h"
#include "RobotIO.h"
#include <unistd.h>


int main() {

	std::cout << "!!!Hello World!!!" << std::endl; // prints !!!Hello World!!!


	RobotThread R1;

	R1.Activate = true;
	R1.ControlThread();

	return 0;
}
