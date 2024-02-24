#include "depend.h"

PID::PID(double kp, double ki, double kd) {
    KP = kp;
    KI = ki;
    KD = kd;
    integral = 0;
    prev_error = 0;
}

PID::PID() {}

void PID::initialize(double kp, double ki, double kd) {
    KP = kp;
    KI = ki;
    KD = kd;
    integral = 0;
    prev_error = 0;
}

double PID::update(double error) {
    double P = error * KP;
	integral += error;
	if (abs(error) < ACCURACY) {
		integral = 0;
	}
    if (error > MAX_I_VAL || error < MIN_I_VAL) {
        integral = 0;
    }
	double I = integral * KI;
	double D = error - prev_error;
	D *= KD;

	return P + I + D;
}

void PID::reset() {
    integral = 0;
    prev_error = 0;
}
