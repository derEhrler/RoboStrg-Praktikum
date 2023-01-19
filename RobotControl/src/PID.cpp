/*
 * PID.cpp
 *
 *  Created on: 25.07.2016
 *      Author: yitao
 */
#include "PID.h"
#include <iostream>
#include <stdio.h>
#include <vector>

using namespace std;
//Constructor without IntegralLimit
PID::PID(std::vector<double> pGain, std::vector<double> iGain, std::vector<double> dSetValueGain, std::vector<double> dCurrentValueGain, double sampleFrequency){
	mProportionalGain = pGain;
	mIntegralGain = iGain;
	mDerivativeSetValueGain = dSetValueGain;
	mDerivativeCurrentValueGain = dCurrentValueGain;
	mSampleFrequency = sampleFrequency;
	mEndpoint = false;

	for (unsigned int i = 0; i < pGain.size(); i++){
		mIntegralSum.push_back(0);
		mIntegralLimit.push_back(0);
		mPreviousCurrentValue.push_back(0);
		mPreviousSetValue.push_back(0);
		merror.push_back(0);
	}
}

//Constructor with IntegralLimit
PID::PID(std::vector<double> pGain, std::vector<double> iGain, std::vector<double> dSetValueGain, std::vector<double> dCurrentValueGain, double sampleFrequency, std::vector<bool> integralLimit, std::vector<double> integralUpperLimit, std::vector<double> integralLowerLimit){
	mProportionalGain = pGain;
	mIntegralGain = iGain;
	mDerivativeSetValueGain = dSetValueGain;
	mDerivativeCurrentValueGain = dCurrentValueGain;
	mSampleFrequency = sampleFrequency;
	mEndpoint = false;

	mIntegralLimit = integralLimit;
	mIntegralUpperLimit = integralUpperLimit;
	mIntegralLowerLimit = integralLowerLimit;

	for (unsigned int i = 0; i < pGain.size(); i++){
		mIntegralSum.push_back(0);
		mPreviousCurrentValue.push_back(0);
		mPreviousSetValue.push_back(0);
	}
}
PID::~PID(void){

}

// PID control algorithm
std::vector<double> veloUpperLimit {36,36,36,48,48,48};
std::vector<double> veloLowerLimit {-36,-36,-36,-48,-48,-48};
std::vector<double> posUpperLimit {10,310,341,10,10,10};
std::vector<double> posLowerLimit {-10,50,19,-10,-10,-10};



std::vector<double> PID::calculate(std::vector<double> setValue,std::vector<double> currentValue){
	std::vector<double> u (mProportionalGain.size());
	mIntegralUpperLimit = {200,200,200,200,200,200};
	mIntegralLowerLimit = {-200,-200,-200,-200,-200,-200};
	for (unsigned int i = 0; i < mProportionalGain.size(); i++){ // i ist Anzahl an Gelenken
		// Gelenkwinkelbegrenzung
		if(setValue[i] < posLowerLimit[i])
			setValue[i] = posLowerLimit[i];
		if(setValue[i] > posUpperLimit[i])
			setValue[i] = posUpperLimit[i];

		// Complete the controller here:
		merror[i] = setValue[i] - currentValue[i];
		mIntegralSum[i] += merror[i];
		if(mIntegralSum[i] < mIntegralLowerLimit[i])
			mIntegralSum[i] = mIntegralLowerLimit[i];
		if(mIntegralSum[i] > mIntegralUpperLimit[i])
			mIntegralSum[i] = mIntegralUpperLimit[i];


		u[i] = mProportionalGain[i] * merror[i] //  P
			 + mIntegralGain[i] * mIntegralSum[i] * (1/mSampleFrequency) // I
			 - mDerivativeCurrentValueGain[i] * (currentValue[i] - mPreviousCurrentValue[i]) * mSampleFrequency // D_y
			 + mDerivativeSetValueGain[i] * (setValue[i] - mPreviousSetValue[i]) * mSampleFrequency; // D_w

		if(u[i] > veloUpperLimit[i])
			u[i] = veloUpperLimit[i];
		if(u[i] < veloLowerLimit[i])
			u[i] = veloLowerLimit[i];
		// cout << merror[i] << " ";
	}

	mPreviousCurrentValue = currentValue;
	mPreviousSetValue = setValue;
	//cout << endl;
	return u;
}

// set previous values for derivatives
void PID::setPreviousCurrentValue(std::vector<double> previousCurrentValue){
	mPreviousCurrentValue = previousCurrentValue;
}

void PID::setPreviousSetValue(std::vector<double> previousSetValue){
	mPreviousSetValue = previousSetValue;
}

std::vector<double> PID::getError(){
	return merror;
}



