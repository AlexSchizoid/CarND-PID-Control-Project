#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  typedef enum {
	COEFF_KP = 0,
	COEFF_KI = 1,
	COEFF_KD = 2
  } coeffs_t; 
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  void addValueToParameter(unsigned int indexParam, double value);
  void twiddle(double cte);

  std::vector<double> dp;
  double mSmallestErr;
  unsigned int mCurrentSample;
  unsigned int mIndexP;
  bool mRaised;
  bool mLowered;
  double mTotalErrorOnLap;
  
};

#endif /* PID_H */
