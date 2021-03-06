#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    total_error = 0.0;
}

void PID::UpdateError(double cte, double dt) {
    d_error = (cte - p_error) / dt;
    p_error = cte;
    i_error += cte;

    // square the cross track error to ensure total_error is always positive
    total_error += cte * cte;
}

double PID::TotalError() {
  return total_error;
}

double PID::CalculateControlValue() {
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}
