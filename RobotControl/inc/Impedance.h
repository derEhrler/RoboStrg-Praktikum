/*
 * Impedance.h
 *
 *  Created on: 26.07.2016
 *      Author: yitao
 */

#ifndef IMPEDANCE_H_
#define IMPEDANCE_H_

#include <vector>

class Impedance{
public:
	Impedance(std::vector<double> spring,std::vector<double> damper,std::vector<double> mass, std::vector<double> pGain, std::vector<double> iGain, std::vector<double> dGain,double sampleFrequency);
	~Impedance();

	void setInitialValues(std::vector<double> currentPosition);

	std::vector<double> calculate(std::vector<double> setPosition, std::vector<double> currenPosition, std::vector<double> currentForce);


private:
	std::vector<double> mSpringGain;
	std::vector<double> mDamperGain;
	std::vector<double> mMassGain;

	std::vector<double> mProportionalGain;
	std::vector<double> mIntegralGain;
	std::vector<double> mDerivativeGain;

	double mSampleFrequency;

	std::vector<double> u;
	std::vector<double> impedance;
	bool mEndpoint;

	std::vector<double> mPositionError;
	std::vector<double> mPositionErrorVelocity;
	std::vector<double> mPositionErrorAcceleration;
	std::vector<double> mPositionPreviousSet;
	std::vector<double> mPositionPreviousCurrentValue;
	std::vector<double> mPositionPreviousErrorVelocity;

	std::vector<double> mForcePreviousError;
	std::vector<double> mForceErrorSum;

	std::vector<double> mForceError;
	std::vector<double> mForceDerivativeError;
};



#endif /* IMPEDANCE_H_ */
