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
static double NORM_RAD(double deg) {
    while (deg < 0) deg += 2 * PI;
    while (deg >= 2 * PI) deg -= 2 * PI;
    return deg;
}
/// @brief Converts standrad angles to thier bearings due north
/// @param deg standrad in degrees
/// @return bearing in degrees
static double standrad_to_bearing(double deg) {
    double b = 90 - deg;
    if (b >= 0) return b;
    else return b + 360;
}
/// @brief Converts standrad angles to thier bearings due north
/// @param rad standrad in radians
/// @return bearing in radians
static double standrad_to_bearing_rad(double rad) {
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
double Location::calc_theta_orient()
{
    return old_t + (rel_l - rel_r) / (ROBOT_WIDTH);
}
VectorXD<2> Location::local_offset()
{
    VectorXD<2> v;
    if (ARE_SAME(new_t, old_t)) {
        v.setIndex(0, rel_c);
        v.setIndex(1, rel_r);
    }
    else {
        v.setIndex(0, rel_c / (new_t - old_t) + 9.251969);
        v.setIndex(1, rel_r / (new_t - old_t) + (ROBOT_WIDTH / 2));
        v.mult(2 * sin(old_t / 2));
    }
    return v;
}
double Location::avg_orient() { 
    return old_t + (new_t - old_t) / 2; 
}
VectorXD<2> Location::global_offset(VectorXD<2> delta_dl) {
    double r = sqrt((*x) * (*x) + (*y) * (*y));
    double theta = atan2(*y, *x) - avg_orient();
    
    double x = r * cos(theta);
    double y = r * sin(theta);
    
    VectorXD<2> vect(x, y);
    return vect;
}

std::vector<double> Location::update() {
    // update variables:
    rel_l = robot->left_abs_dist() - old_l;
	rel_r = robot->right_abs_dist() - old_r;
	rel_c = robot->center_abs_dist() - old_c;
	new_t = calc_theta_orient();

    // calculate and update coordinates:
    // VectorXD<2> vect = local_offset();
    // double avgOr = avg_orient();
    // VectorXD<2> change = global_offset(vect);
    // *theta += avgOr;

    std::vector<double> arr = {
        (((rel_l + rel_r) / 2 * sin(robot->get_abs_angle(true)))) / 0.08203342547, // conversion factor...
        (((rel_l + rel_r) / 2 * cos(robot->get_abs_angle(true)))) / 0.08203342547,
    };

    // save new vars to cache:
    old_l = robot->left_abs_dist();
	old_r = robot->right_abs_dist();
	old_c = robot->center_abs_dist();
	old_t = new_t;

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


static double toTheta(double x, double y, Robot& robot) {
    double ret = atan(y/x);
    ret = robot.radians_to_degrees(ret);
    if (ret < 0)
    {
        ret = 360 + ret;
    }
    return ret;
}
std::vector<double> Location::updatePID(Waypoint& goal) {
    double error_x = *x - goal.x;
    double error_y = *y - goal.y;
    
    int c = 1;
    if (*x > goal.x || *y > goal.y) c *= -1;
    error = c * sqrt(error_x*error_x + error_y*error_y);

    error_turn = robot->get_abs_angle() - toTheta(goal.x, goal.y);

    // pid stuff:
    double power = PID(error, integral, prev_error, goal, false);
    double turn = PID(error_turn, integral_turn, prev_error_turn, goal, true);

    std::vector<double> v = {power - turn, power + turn};
    /////////////

    prev_error = error;
    prev_error_turn = error_turn;

    return v;
}
