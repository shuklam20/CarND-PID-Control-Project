#include "PID.h"
#include <iostream>
#include <string>
using std::string;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
	// Initialize PID coefficients (and errors, if needed)
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
//	Twiddle(0.05);
	prev_cte = 0;
	accum_cte = 0;
}

void PID::UpdateError(double cte) {
	accum_cte += cte;
	// Update PID errors based on CTE.
	p_error = cte;
	i_error = accum_cte;
	d_error = cte - prev_cte;
	prev_cte = cte;
}

double PID::TotalError() {
	// Calculate and return the total error
	return -(Kp * p_error + Kd * d_error + Ki * i_error);
}

void PID::Twiddle(double tol) {
	double dp[3] = {1, 1, 1};
	Kp = 0;
	Ki = 0;
	Kd = 0;

	double best_error = TotalError();
	int it = 0;
	double err;
	int n = sizeof(dp) / sizeof(dp[0]);
	while (arr_sum(dp, n) > tol) {
		Kp += dp[0]; Ki += dp[1]; Kd += dp[2];
		err = TotalError();

		if (err < best_error) {
			best_error = err;
			dp[0] *= 1.1; dp[1] *= 1.1; dp[2] *= 1.1;
		}
		else {
			Kp -= 2 * dp[0]; Ki -= 2 * dp[1]; Kd -= 2 * dp[2];
			std::cout << "value of Kp " << Kp << " Ki " << Ki << " and Kd " << Kd << std::endl;

			err = TotalError();
			if (err < best_error) {
				best_error = err;
				dp[0] *= 1.1; dp[1] *= 1.1; dp[2] *= 1.1;
			}
			else {
				Kp += dp[0]; Ki += dp[1]; Kd += dp[2];
				dp[0] *= 0.9; dp[1] *= 0.9; dp[2] *= 0.9;
			}
		}
	    it += 1;
	}
//	std::cout << "value of Kp " << Kp << " Ki " << Ki << " and Kd " << Kd << std::endl;
//	std::cout << "total_it " << it << std::endl;
}



double PID::arr_sum(double dp[], int n) {
	double dp_sum = 0.0;
	for (int i = 0; i < n; i++) {
		dp_sum += dp[i];
	}
	return dp_sum;
}

