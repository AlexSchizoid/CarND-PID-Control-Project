#include "PID.h"
#include <limits>
#include <iostream>
#include <cmath>

//#define use_twiddle
using namespace std;

PID::PID() : p_error(0), i_error(0), d_error(0), Kp(0), Ki(0), Kd(0), mCurrentSample(1), mCurrentParameter(0), mRaised(false), mLowered(false), mLapError(0) {
 dp = { 1, 1, 1};
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	dp = { 0.15 * Kp, 0.15 * Ki, 0.15 * Kd};
	mCurrentParameter = 0;
	mSmallestErr = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
	double previous_error = p_error;
	d_error = cte - previous_error;
	p_error = cte;
	i_error += cte;
	twiddle(cte);	
	
}

void PID::twiddle(double cte) {
	//activate/deactivate twiddle
	#ifndef use_twiddle
	return;
	#endif	
	cout << "Current sample: " << mCurrentSample << endl;
	
	//the error we compare is the error on a full lap
	//considering we make a change in one of the parameters then we let the system stabilize for a number of samples
	//i consider a full lap to be arround 3300 laps at my current chosen speed
	if (mCurrentSample - 300 > 0) {
		mLapError += std::pow(cte, 2);
	}
	
	//do a step of the twiddle algo after every lap
	//we use mRaised and mLowered do determine the current step in the algo
	//a "sort-of" state machine

	//we try to raise the value, lower the value and compare errors
	if (mCurrentSample == 3300) {
		if (!mRaised && !mLowered) { 
			//first step try to raise the value of the parameter by adding dp
			addValueToParameter(mCurrentParameter, dp[mCurrentParameter]);
			mRaised = true;
		}
		else if (mRaised && !mLowered) {
			//if there is improvement move on to the next paramater and raise dp a little
			if (mLapError < mSmallestErr) {
				mSmallestErr = mLapError;
				dp[mCurrentParameter] *= 1.1;
				mRaised = false;
				mLowered = false;
				mCurrentParameter = (mCurrentParameter + 1) % 3;
			}
			else {
				//no improvement then try to lower the value of the parameter by subtracting twice dp
				addValueToParameter(mCurrentParameter, -2 * dp[mCurrentParameter]);
				mLowered = true;
			}
		}
		else if (mRaised && mLowered) {
			//improvement so raise dp a little more and move on
			if (mLapError < mSmallestErr) {
				mSmallestErr = mLapError;
				dp[mCurrentParameter] *= 1.1;
			}
			else {
				//if no improvement is detected then we tweak the dp value and go back to the initial value of the parameter
				addValueToParameter(mCurrentParameter, dp[mCurrentParameter]);
				dp[mCurrentParameter] *= 0.9;
			}
			//back to initial step of twiddle
			mRaised = false;
			mLowered = false;
			mCurrentParameter = (mCurrentParameter + 1) % 3;
		}
		//new lap and twiddle pass
		mLapError = 0;
		mCurrentSample = 0;
	}
	else
		++mCurrentSample;
	
}

double PID::TotalError() {
	std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;
	return Kp * p_error + Ki * i_error + Kd * d_error;
}

void PID::addValueToParameter(unsigned int parameter, double value) {
	switch(parameter) {
	case COEFF_KP:
		Kp += value;
		break;
	case COEFF_KI:
		Ki += value;
		break;
	case COEFF_KD: 
		Kd += value;
		break;
	default:
		break;
	};
}
