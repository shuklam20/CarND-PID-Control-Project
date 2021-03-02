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
    if (use_twiddle) {
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
    }
    //std::cout << "value of Kp " << Kp << " Ki " << Ki << " and Kd " << Kd << std::endl;
	// Calculate and return the total error
	return (Kp * p_error + Kd * d_error + Ki * i_error);
}

void PID::Twiddle(double tol, double cte) {
    use_twiddle = true;
    int n = sizeof(dp) / sizeof(dp[0]);
    for (int i=0; i<n; i++) {
        dp[i] = 1;
        p[i] = 0;
    }
    
    double best_error = TotalError();
	int it = 0;
	double err;

	while (arr_sum(dp, n) > tol) {
        for (int i=0; i<n; i++) {
            p[i] += dp[i];
            UpdateError(cte);
            err = TotalError();

            if (err < best_error) { // There was some improvement
                best_error = err;
                dp[i] *= 1.1;
            }
            else { // No improvement
                p[i] -= 2 * dp[i];
                UpdateError(cte);
                err = TotalError();
                
                if (err < best_error) { // There was an improvement
                    best_error = err;
                    dp[i] *= 1.1;
                }
                else { // No improvement
                    Kp += dp[0];
                    dp[i] *= 0.9;
                }
            }
        }
        it += 1;
	}
	std::cout << "total_it " << it << std::endl;
}



double PID::arr_sum(double dp[], int n) {
	double dp_sum = 0.0;
	for (int i = 0; i < n; i++) {
		dp_sum += dp[i];
	}
	return dp_sum;
}

