/*
 * PID.h
 *
 *  Created on: 25.07.2016
 *      Author: yitao
 */

#ifndef PID_H_
#define PID_H_

#include <vector>

class PID{
public:
	PID(std::vector<double> pGain, std::vector<double> iGain, std::vector<double> dSetValueGain, std::vector<double> dCurrentValueGain, double sampleFrequency);
	PID(std::vector<double> pGain, std::vector<double> iGain, std::vector<double> dSetValueGain, std::vector<double> dCurrentValueGain, double sampleFrequency, std::vector<bool> integralLimit, std::vector<double> integralUpperLimit, std::vector<double> integralLowerLimit);
	~PID();

	std::vector<double> calculate(std::vector<double> setValue,std::vector<double> currentValue);

	void setPreviousCurrentValue(std::vector<double>);
	void setPreviousSetValue(std::vector<double>);

	std::vector<double> getError();
private:
	std::vector<double> merror;
	std::vector<double> mProportionalGain;
	std::vector<double> mIntegralGain;
	std::vector<double> mDerivativeSetValueGain;
	std::vector<double> mDerivativeCurrentValueGain;

	std::vector<double> mIntegralSum;
	std::vector<bool> 	mIntegralLimit;
	std::vector<double> mIntegralUpperLimit;
	std::vector<double> mIntegralLowerLimit;
	std::vector<double> mPreviousCurrentValue;
	std::vector<double> mPreviousSetValue;
	double mSampleFrequency;
	bool mEndpoint;
};

#endif /* PID_H_ */
