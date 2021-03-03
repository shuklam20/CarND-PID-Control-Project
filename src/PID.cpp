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
    
	prev_cte = 0.0;
	accum_cte = 0.0;
    best_error = 0.0;
    current_err = 0.0;
    n = sizeof(dp) / sizeof(dp[0]);
    twiddle_call = 0;
}

void PID::UpdateParams(double K_, int i) {
    // Update PID coefficients while using twiddle
    switch(i) {
        case 0:
            Kp = K_;
            break;
        case 1:
            Ki = K_;
            break;
        case 2:
            Kd = K_;
            break;
    }
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

void PID::Twiddle(double tol, double cte) {
    twiddle_call += 1;
    current_err += (cte * cte);
    current_err /= twiddle_call;
    best_error = (cte * cte);
    for (int i=0; i<n; i++) {
        dp[i] = 1;
        p[i] = 0;
    }
    
	int it = 0;
	while (arr_sum(dp, n) > tol) {
//        std::cout << "arr_sum(dp, n)" << arr_sum(dp, n) << std::endl;
//        std::cout << "current_err " << current_err << "best_error " << best_error << std::endl;
        for (int i=0; i<n; i++) {
            p[i] += dp[i];
            UpdateParams(p[i], i);
            
            if (current_err < best_error) { // There was some improvement
                std::cout << "loop 1" << std::endl;
                best_error = current_err;
                dp[i] *= 1.1;
            }
            else { // No improvement
                std::cout << "loop 2" << std::endl;
                p[i] -= 2 * dp[i];
                UpdateParams(p[i], i);
                
                if (current_err < best_error) { // There was an improvement
                    std::cout << "loop 2a" << std::endl;
                    best_error = current_err;
                    dp[i] *= 1.1;
                }
                else { // No improvement
                    std::cout << "loop 2b" << std::endl;
                    p[i] += dp[0];
                    dp[i] *= 0.9;
                }
            }
        }
        it += 1;
	}
//    std::cout << "value of Kp " << Kp << " Ki " << Ki << " and Kd " << Kd << std::endl;
	std::cout << "total_it " << it << std::endl;
}

double PID::arr_sum(double dp[], int n) {
	double dp_sum = 0.0;
	for (int i = 0; i < n; i++) {
		dp_sum += dp[i];
	}
	return dp_sum;
}

