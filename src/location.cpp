#include "depend.h"
#include "vectorxd.h"

template <int N>
VectorXD<N>::VectorXD(double x, double y) {
    if (N != 2) {
        pros::lcd::print(1, "ERROR, your vector is %i rows, but to use this constructor it must be 2,", size());
        TERMINATE();
    }
    c_arr[0] = x;
    c_arr[1] = y;
}
template <int N>
VectorXD<N>::VectorXD() {   
    for (int i = 0; i < N; ++i) {
        c_arr[i] = 0;
    }
}
template <int N>
double VectorXD<N>::getIndex(int i) {
    if (i >= size()) {
        pros::lcd::print(1, "ERROR, index is out of bounds [%i out of %i]", i, size());
        TERMINATE();
    }
    return c_arr[i];
}
template <int N>
void VectorXD<N>::setIndex(int i, double val) {
    if (i >= size()) {
        pros::lcd::print(1, "ERROR, index is out of bounds [%i out of %i]", i, size());
        TERMINATE();
    }
    c_arr[i] = val;
}
template <int N>
VectorXD<N> VectorXD<N>::mult(double X) {
    VectorXD<N> v;
    for (int i = 0; i < N; ++i) c_arr[i] *= X;
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::div(double X) {
    VectorXD<N> v();
    for (int i = 0; i < N; ++i) c_arr[i] /= X;
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::add_by_vect(VectorXD<N> Vect) {
    VectorXD<N> v();
    for (int i = 0; i < N; ++i) v[i] = c_arr[i] + Vect[i];
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::sub_by_vect(VectorXD<N> Vect) {
    VectorXD<N> v();
    for (int i = 0; i < N; ++i) v[i] = c_arr[i] - Vect[i];
    return v;
}
template <int N>
int VectorXD<N>::size() const { return N; }


















int Location::normalize(double deg) {
    int ret = (int)deg;
    while (ret < 0) ret += 360;
    while (ret >= 360) ret -= 360;
    return ret;
}

static double NORM_RAD(double deg) { // normalize radians from 0 - 2 pi
    while (deg < 0) deg += 2 * PI;
    while (deg >= 2 * PI) deg -= 2 * PI;
    return deg;
}
static double standrad_to_bearing(double deg) { // Converts standrad angles to thier bearings due north 
    double b = 90 - deg;
    if (b >= 0) return b;
    else return b + 360;
}
static double standrad_to_bearing_rad(double rad) { // Converts standrad angles to thier bearings due north
    rad = NORM_RAD(rad);
    double b = PI / 2 - rad;
    if (b >= 0) return b;
    else return b + 2 * PI;
}

void Location::initialize(Robot& r) {
    robot = &r;
    x = &r.x;
    y = &r.y;
}

std::vector<double> Location::update() {
    // update variables:
    rel_l = robot->left_abs_dist() - old_l;
	rel_r = robot->right_abs_dist() - old_r;
    robot->theta = normalize(robot->get_abs_angle());

    std::vector<double> arr = {
        (((rel_l + rel_r) / 2 * sin(robot->theta))) / 0.08203342547, // conversion factor...
        (((rel_l + rel_r) / 2 * cos(robot->theta))) / 0.08203342547,
    };

    // save new vars to cache:
    old_l = robot->left_abs_dist();
	old_r = robot->right_abs_dist();

    return arr;
}

double Location::P(double error, bool isturn) { 
	return error * (isturn ? TURN_KP : POWER_KP); 
}
double Location::I(double error, double& integral, Waypoint& goal, bool isturn) {
	integral += error;
	if (ARE_SAME(goal.x, *x) && ARE_SAME(goal.y, *y) || ARE_SAME(error, 0)) {
		integral = 0;
	}
	double max = (isturn ? TURN_ERROR_MAX : POWER_ERROR_MAX);
	double min = (isturn ? TURN_ERROR_MIN : POWER_ERROR_MIN);
	if (max < error || min > error) {
		integral = 0;
	}
	return integral * (isturn ? TURN_KI : POWER_KI);
}
double Location::D(double& prev_error, double error, bool isturn) {
	return (error - prev_error) * (isturn ? TURN_KD : POWER_KD);
}

double Location::PID(double error, double& integral, double& prev_error, Waypoint& goal, bool isturn) {
	return P(error, isturn) + I(error, integral, goal, isturn) + D(prev_error, error, isturn);
}


static double toTheta(double x, double y, Robot* robot) {
    double ret = atan(y/x);
    ret = robot->radians_to_degrees(ret);
    if (x>=0 && y>=0) {
        return ret;
    }
    if (x>=0 && y<0) {
        return 360 + ret;
    }
    if (x < 0 && y >= 0) {
        return 180 + ret;
    }
    return 180 + ret;
}

static double angleDifference(double start, double end) {
    double red = ABS(end-start);
    if (red >= 180) {
        return red - 360;
    }
    return red;
}

std::vector<double> Location::updatePID(Waypoint& goal) {
    double error_x = *x - goal.x;
    double error_y = *y - goal.y;
    
    int c = 1;
    if (*x > goal.x || *y > goal.y) c *= -1;
    error = c * sqrt(error_x*error_x + error_y*error_y);

    error_turn = angleDifference(robot->get_abs_angle(), toTheta(goal.x, goal.y, robot)) * PIVOT_P_TO_PERP_ODOM;

    // pid stuff:
    double power = PID(error, integral, prev_error, goal, false);
    double turn = PID(error_turn, integral_turn, prev_error_turn, goal, true);

    std::vector<double> v = {power - turn, power + turn};
    /////////////

    prev_error = error;
    prev_error_turn = error_turn;

    return v;
}
