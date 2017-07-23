#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->p_error = 0;
}

void PID::UpdateError(double cte) {
	this->p_error = cte;
}

double PID::TotalError() {
	return -Kp * this->p_error;
}

