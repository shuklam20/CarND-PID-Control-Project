#ifndef PID_H
#define PID_H

class PID {
	public:
	/**
	* Constructor
	*/
	PID();

	/**
	* Destructor.
	*/
	virtual ~PID();

	/**
	* Initialize PID.
	* @param (Kp_, Ki_, Kd_) The initial PID coefficients
	*/
	void Init(double Kp_, double Ki_, double Kd_);

	/**
	* Update the PID error variables given cross track error.
	* @param cte The current cross track error
	*/
	void UpdateError(double cte);

	/**
	* Calculate the total PID error.
	* @output The total PID error
	*/
	double TotalError();

	/**
	* Calculate Kp, Kd and Ki values
	* given a tolerance parameter
	*/
	void Twiddle(double tol);

	/**
	* Calculate the sum of a given array
	* given its length and elements
	*/
	double arr_sum(double dp[], int n);


 private:
	/**
	* PID Errors
	*/
	double p_error;
	double i_error;
	double d_error;

	/**
	* PID Coefficients
	*/
	double Kp;
	double Ki;
	double Kd;

	/**
	* previous CTE and sum of CTEs
	*/
	double prev_cte;
	double accum_cte;
};

#endif  // PID_H
