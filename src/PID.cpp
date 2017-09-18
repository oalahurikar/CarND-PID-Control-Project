#include "PID.h"

/*
* PID CLASS
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)  {
    this->Kp = Kp; // proportional coefficient
    this->Ki = Ki; // integral coefficient
    this->Kd = Kd; // differential coefficient

    p_error = 1.0;  // proportional cte
    i_error = 0.0; // integral cte
    d_error = 0.0; // differential cte
}

void PID::UpdateError(double cte) {
// Pass CTE through integral, differencial and praportional terms

    d_error = cte - p_error; // diff_cte = cte - prev_cte -> p_error is equal to prev_cte
    p_error = cte; // update p_error
    i_error += cte; // add i_error
}

double PID::TotalError() {
    // Return total error to control car
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}