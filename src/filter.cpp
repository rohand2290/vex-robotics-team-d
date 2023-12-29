#include "depend.h"

ComplementaryFilter::ComplementaryFilter(double a) {
    if (a > 1.0) alpha = 1.0;
    else if (a < 0.0) alpha = 0.0;
    else alpha = a;
}

double ComplementaryFilter::get_alpha() { return alpha; }

void ComplementaryFilter::set_alpha(double a) {
    if (a > 1.0) alpha = 1.0;
    else if (a < 0.0) alpha = 0.0;
    else alpha = a;
}

double ComplementaryFilter::compute(double sensor1, double sensor2) {
    return alpha * sensor1 + (1 - alpha) * sensor2;
}

Kalman1VFilter::Kalman1VFilter(double EM, double INIT): errorMea(EM), init(INIT) {}

void Kalman1VFilter::set_error_mea(double val) {
    errorMea = val;
}

void Kalman1VFilter::set_init_val(double val) {
    init = val;
}

double Kalman1VFilter::compute(double sensor) {
    KG = errorEst / (errorEst + errorMea);
    est = oldEst + KG * (sensor - oldEst);
    errorEst = (1 - KG) * oldErrorEst;

    oldErrorEst = errorEst;
    oldEst = est;

    return est;
}