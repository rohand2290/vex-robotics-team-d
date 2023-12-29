#include "depend.h"
#include "location.h"

double Location::right_abs_dist()
{ // in terms of inches
    Items& items = robot->items;
    double normal = items.right1->get_position() + items.right2->get_position() + items.right3->get_position();
    return normal / 3;
}

double Location::left_abs_dist()
{ // in terms of inches
    Items& items = robot->items;
    double normal = items.left1->get_position() + items.left2->get_position() + items.left3->get_position();
    return normal / 3;
}

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
}

std::vector<double> Location::update() {
    rel_l = left_abs_dist() - old_l;
	rel_r = right_abs_dist() - old_r;
    // rel_th = robot->get_abs_angle() - old_th;
    robot->theta = robot->get_abs_angle();

    std::vector<double> arr = {
        ((rel_l + rel_r) / 2 * sin(robot->degrees_to_radians(robot->theta))) / 48, // conversion factor...
        ((rel_l + rel_r) / 2 * cos(robot->degrees_to_radians(robot->theta))) / 48,
    };

    robot->x += arr[0];
    robot->y += arr[1];

    // save new vars to cache:
    old_l = left_abs_dist();
	old_r = right_abs_dist();
    // old_th = robot->get_abs_angle();

    return arr;
}

double Location::P(double error, bool isturn) { 
	return error * (isturn ? TURN_KP : POWER_KP); 
}
double Location::I(double error, double& integral, Waypoint& goal, bool isturn) {
	integral += error;
	if (ARE_SAME(goal.right, right_abs_dist()) && ARE_SAME(goal.left, left_abs_dist()) || ARE_SAME(error, 0)) {
		integral = 0;
	}
    double max = (isturn ? TURN_ERROR_MAX : POWER_ERROR_MAX);
	double min = (isturn ? TURN_ERROR_MIN :POWER_ERROR_MIN);
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
    if (x == 0) x = 0.000001;
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

// look at its code to figure out what it does...
static double modifier(double X) {
    if (X > 180) return (180 - X);
    return X;
}
std::vector<double> Location::updatePID(Waypoint& goal) {
    double error_x = goal.right - robot->x;
    double error_y = goal.left - robot->y;

    CartesianLine robot_line(
        tan(robot->degrees_to_radians(90 - robot->theta)), robot->x, robot->y
    );
    CartesianLine goal_line(
        robot_line.get_perp(robot_line.get_slope()),
        goal.right,
        goal.left
    );
    int c = 1;
    if (
        goal_line.is_above(robot->x, robot->y) == goal_line.is_bellow(robot->x, robot->y)
    ) c = 0;
    if (goal_line.is_above(robot->x, robot->y)) c = -1;

    error = c * sqrt(error_x*error_x + error_y*error_y);

    error_l = normalize(90 - toTheta(goal.right, goal.left, robot)) - modifier(robot->theta);

    // pid stuff:
    double power = pid(error, integral, prev_error, goal, false);
    double turn = pid(error_l, integral_l, prev_error_l, goal, true);
    
    std::vector<double> v = {power - turn, power + turn};

    if (ARE_SAME(error, 0) && ARE_SAME(error_l, 0)) timer++;

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






IMULocation::IMULocation(Robot& r): robot(r), items(r.items) {
    filtx = new Kalman1VFilter(ERROR_MEASUREMENT, 0);
    filty = new Kalman1VFilter(ERROR_MEASUREMENT, 0);
    x = r.x;
    y = r.y;
}

void IMULocation::compute() {
    pros::c::imu_accel_s_t raw_val = items.imu->get_accel();
    // accelerometer vals...
    VectorXD<2> acc(raw_val.y, raw_val.x);
    acc.rotate(-robot.theta);
    // double integration...
    prev_vel = prev_vel.add_by_vect(acc);
    prev_dis = prev_dis.add_by_vect(prev_vel).mult(G / 100000);
    // filtering...
    prev_dis.setIndex(0, filtx->compute(prev_dis.getIndex(0)));
    prev_dis.setIndex(1, filty->compute(prev_dis.getIndex(1)));
}

double IMULocation::getX() {
    return prev_dis.getIndex(0);
}

double IMULocation::getY() {
    return prev_dis.getIndex(1);
}

IMULocation::~IMULocation() {
    delete filtx;
    delete filty;
}
