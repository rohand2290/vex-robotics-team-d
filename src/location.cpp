#include "depend.h"
#include "location.h"

double Location::normalize(double deg) {
    double ret = deg;
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
    reset_all();
    // leftPID.set_exit_condition(MAX_ALLOWED_ERROR_TIME, MAX_ALLOWED_ERROR);
    // rightPID.set_exit_condition(MAX_ALLOWED_ERROR_TIME, MAX_ALLOWED_ERROR);
}

std::vector<double> Location::update() {
    rel_l = robot->left_abs_dist() - old_l;
	rel_r = robot->right_abs_dist() - old_r;
    rel_th = robot->get_abs_angle() - old_th;
    robot->theta = robot->get_abs_angle();

    // if (rel_th == 0) rel_th = 0.000000001;

    // double mag = ((rel_l + rel_r) / (rel_th)) * sin(robot->degrees_to_radians(rel_th) / 2);
    // std::vector<double> arr = {
    //     dx * sin(robot->degrees_to_radians(robot->theta)), // / 0.08203342547,
    //     dy * cos(robot->degrees_to_radians(robot->theta)) // / 0.08203342547
    // };

    std::vector<double> arr = {
        ((rel_l + rel_r) / 2 * sin(robot->degrees_to_radians(robot->theta)))* 0.787402, // conversion factor...
        ((rel_l + rel_r) / 2 * cos(robot->degrees_to_radians(robot->theta)))* 0.787402,
    };

    // robot->x += arr[0];
    // robot->y += arr[1];

    // save new vars to cache:
    old_l = robot->left_abs_dist();
	old_r = robot->right_abs_dist();
    old_th = robot->get_abs_angle();

    return arr;
}

double Location::P(double error, bool isturn) { 
	return error * (isturn ? TURN_KP : POWER_KP); 
}
double Location::I(double error, double& integral, Waypoint& goal, bool isturn) {
	integral += error;
	if (ARE_SAME(goal.right, robot->right_abs_dist()) && ARE_SAME(goal.left, robot->left_abs_dist()) || ARE_SAME(error, 0)) {
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

double Location::pid(double error, double& integral, double& prev_error, Waypoint& goal, bool isturn) {
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
    error = goal.right - robot->right_abs_dist();
    error_l = goal.left - robot->left_abs_dist();

    // pid stuff:
    double right = pid(error, integral, prev_error, goal, false);
    double left = pid(error_l, integral_l, prev_error_l, goal, false);
    
    std::vector<double> v = {right, left};

    if (ARE_SAME(error, 0) && ARE_SAME(error_l, 0)) timer++;

    // if (rightPID.get_target() != goal.right) rightPID.set_target(goal.right);
    // if (leftPID.get_target() != goal.left) leftPID.set_target(goal.left);
    // /////////// NOTE: temporarily, x is right odom goal and y is left goal.
    // double l = leftPID.compute(robot->left_abs_dist());
    // double r = rightPID.compute(robot->right_abs_dist());

    // std::vector<double> v = {r, l};

    return v;
}

void Location::reset_all()
{
    robot->items.left2->tare_position();
	robot->items.right2->tare_position();
    robot->items.left1->tare_position();
	robot->items.right1->tare_position();
    robot->items.left3->tare_position();
	robot->items.right3->tare_position();
}

bool Location::is_running() {
    return timer < MIN_ALLOWED_ERROR_TIME;
}
