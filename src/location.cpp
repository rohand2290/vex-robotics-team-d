#include "depend.h"
#include "location.h"

double Location::right_abs_dist()
{ // in terms of inches
    Items& items = robot->items;
    double normal = items.right1->get_position() + items.right2->get_position() + items.right3->get_position();
    return (normal / 3); // * WHEEL_C;
}

double Location::left_abs_dist()
{ // in terms of inches
    Items& items = robot->items;
    double normal = items.left1->get_position() + items.left2->get_position() + items.left3->get_position();
    return (normal / 3); // * WHEEL_C;
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

    double mag = rel_l > rel_r ? rel_r : rel_l;

    std::vector<double> arr = {
        ((rel_l + rel_r) / 2 * sin(robot->degrees_to_radians(robot->theta))) / 48, // conversion factor...
        ((rel_l + rel_r) / 2 * cos(robot->degrees_to_radians(robot->theta))) / 48,
    };

    // std::vector<double> arr = {
    //     (mag * sin(robot->degrees_to_radians(robot->theta))) / 48, // conversion factor...
    //     (mag * cos(robot->degrees_to_radians(robot->theta))) / 48,
    // };

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
    double max = (isturn ? TURN_ERROR_MAX : POWER_ERROR_MAX);
	double min = (isturn ? TURN_ERROR_MIN :POWER_ERROR_MIN);
    if (ARE_SAME(error, 0)) integral = 0;
	if (max < error || min > error) {
		integral = 0;
	}
	return integral * (isturn ? TURN_KI : POWER_KI);
}
double Location::D(double& prev_error, double error, bool isturn) {
	double tor = (error - prev_error) * (isturn ? TURN_KD : POWER_KD);
    prev_error = error;
    return tor;
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

static double angleDifference(double start, double end, double goal) {
    double red = (end-start);
    if (red >= 180) {
        return red - 360;
    }
    return red;
}

// look at its code to figure out what it does...
static double absolute(double X) {
    if (X < 0) return -X;
    else return X;
}
// static void move_rel(double r, double l, Items& items) {
//     double left = l / WHEEL_C, right = r / WHEEL_C;
//     items.left1->move_relative(l, APPROACH_SPEED);
//     items.left2->move_relative(l, APPROACH_SPEED);
//     items.left3->move_relative(l, APPROACH_SPEED);
//     items.right1->move_relative(l, APPROACH_SPEED);
//     items.right2->move_relative(l, APPROACH_SPEED);
//     items.right3->move_relative(l, APPROACH_SPEED);
//     while (
//         !((items.left1->get_position() < l + MIN_ALLOWED_ERROR) && (items.left1->get_position() > l - MIN_ALLOWED_ERROR)) &&
//         !((items.left2->get_position() < l + MIN_ALLOWED_ERROR) && (items.left2->get_position() > l - MIN_ALLOWED_ERROR)) &&
//         !((items.left3->get_position() < l + MIN_ALLOWED_ERROR) && (items.left3->get_position() > l - MIN_ALLOWED_ERROR)) &&
//         !((items.right1->get_position() < l + MIN_ALLOWED_ERROR) && (items.right1->get_position() > l - MIN_ALLOWED_ERROR)) &&
//         !((items.right2->get_position() < l + MIN_ALLOWED_ERROR) && (items.right2->get_position() > l - MIN_ALLOWED_ERROR)) &&
//         !((items.right3->get_position() < l + MIN_ALLOWED_ERROR) && (items.right3->get_position() > l - MIN_ALLOWED_ERROR)) 
//     ) {
//         pros::delay(2);
//     }
// }
std::vector<double> Location::updatePID(Waypoint& goal, CartesianLine& robot_line, CartesianLine& goal_line, bool turn) {
    
    if (goal.command == "move") {
        // distance PID:

        int dir = 1;
        if (goal.param1 < 0) dir = -1;

        double error_x = goal_line.x - robot->x;
        double error_y = dir * (goal_line.y - robot->y);

        // robot_line.slope = tan(robot->degrees_to_radians(robot->theta));
        // goal_line.slope = robot_line.get_perp(robot_line.get_slope());

        int c = 1;
        if (error_y < 0) c = -1;
        error = c * sqrt(error_x*error_x + error_y*error_y);

        double power = pid(error, integral, prev_error, goal, false);

        std::vector<double> v = {dir * power, dir * power};
        if (absolute(error) < MIN_ALLOWED_ERROR) timer++;
        return v;

    } else if (goal.command == "turn") {
        // turn PID:
        error_l = goal.param1 - robot->items.imu->get_rotation();

        double turn = pid(error_l, integral_l, prev_error_l, goal, true);
        std::vector<double> v = {-turn, turn};
        if (abs(error_l) < MIN_ALLOWED_ERROR_DEG) timer++;

        return v;
    } else if (goal.command == "curve") { //// @TODO
        // curve PID: (first val is mag, second is ending theta)
        double x = sin(robot->degrees_to_radians(goal.param2)) * goal.param1;
        double y = cos(robot->degrees_to_radians(goal.param2)) * goal.param1;
        double t = robot->theta - robot->radians_to_degrees(atan(x / y));
        
        CartesianCircle temp(0, 0, 0);
        double arclen = (goal.param2 * PI * goal.param1) / (360.0 * sin(t / 2));
        double radius = temp.compute_radius(x, y, t);
        std::vector<Point2D> points = temp.find_intersections(x, y, radius);

        // construct circles to follow:
        CartesianCircle c(0, 0, 0);
        {
            // FIX:
            CartesianCircle c1(points[0].x, points[0].y, radius);
            CartesianCircle c2(points[1].x, points[1].y, radius);
            TERMINATE();
            if (
                ABS(standrad_to_bearing(normalize(c1.derivative_at(0))) - robot->theta) >
                ABS(standrad_to_bearing(normalize(c2.derivative_at(0))) - robot->theta)
            ) c = c2;
            else c = c1;
        }        
    
        // align bot at init:
        double init_angle = c.derivative_at(0);
        long ti = 0;
        while (ti < MIN_ALLOWED_ERROR_TIME) {
            // turn PID:
            error_l = angleDifference(robot->theta, init_angle, goal.param1);
            double turn = pid(error_l, integral_l, prev_error_l, goal, true);
            pros::lcd::print(1, "%f", turn);
            std::vector<double> v = {-turn, turn};
            if (ABS(error_l) < MIN_ALLOWED_ERROR_DEG) ti++;
        }
        // start follwing path:
        ti = 0;
        pros::lcd::print(1, "DONE!");
        TERMINATE();
        while (ti < MIN_ALLOWED_ERROR_TIME) {
            double error_x = goal_line.x - robot->x;
            double error_y = goal_line.y - robot->y;
            robot_line.slope = tan(robot->degrees_to_radians(robot->theta));
            goal_line.slope = robot_line.get_perp(robot_line.get_slope());
            int ci = 1;
            if (error_y < 0) ci = -1;
            error = ci * sqrt(error_x*error_x + error_y*error_y);
            double power = pid(error, integral, prev_error, goal, false);

            error_l = angleDifference(robot->theta, c.derivative_at(robot->x), goal.param2);
            double turn = pid(error_l, integral_l, prev_error_l, goal, true);

            std::vector<double> v = {power - turn, power + turn};
            robot->set_both_sides(v[1], v[0]);

            if (error < MIN_ALLOWED_ERROR) ti++;
        }
    }
    
    abs_timer++;

    /*
    // double error_x = goal.right - robot->x;
    // double error_y = goal.left - robot->y;

    // robot_line.slope = tan(robot->degrees_to_radians(robot->theta));

    // int c = 1;
    // if (
    //     goal_line.is_above(robot->x, robot->y) == goal_line.is_bellow(robot->x, robot->y)
    // ) c = 0;
    // if (goal_line.is_above(robot->x, robot->y)) c = -1;

    // error = c * sqrt(error_x*error_x + error_y*error_y);

    // error_l = standrad_to_bearing(toTheta(goal.right, goal.left, robot)) - modifier(robot->theta);

    // // pid stuff:
    // double power = pid(error, integral, prev_error, goal, false);
    // double turn = pid(error_l, integral_l, prev_error_l, goal, true);
    
    // std::vector<double> v = {power - turn, power + turn};

    // if (ARE_SAME(error, 0) && ARE_SAME(error_l, 0)) timer++;

    // return v;
    */
}

void Location::reset_all()
{
    robot->items.left2->tare_position();
	robot->items.right2->tare_position();
    robot->items.left1->tare_position();
	robot->items.right1->tare_position();
    robot->items.left3->tare_position();
	robot->items.right3->tare_position();
    robot->items.imu->tare_rotation();
    robot->items.imu->tare_heading();
    timer = 0;
    error = 0;
    prev_error = 0;
    integral = 0;
    error_l = 0;
    prev_error_l = 0;
    integral_l = 0;
}

bool Location::is_running() {
    if (abs_timer >= MIN_ALLOWED_ERROR_TIMEOUT) return false;
    return timer <= MIN_ALLOWED_ERROR_TIME;
}


#include "vectorxd.cpp"


IMULocation::IMULocation(Robot& r): robot(r), items(r.items) {
    filtx = new Kalman1VFilter(ERROR_MEASUREMENT, 0.001);
    filty = new Kalman1VFilter(ERROR_MEASUREMENT, 0.001);
    x = r.x;
    y = r.y;
}

void IMULocation::compute() {
    pros::c::imu_accel_s_t raw_val = items.imu->get_accel();
    // accelerometer vals...
    VectorXD<2> acc(raw_val.y, raw_val.x);
    acc.setIndex(0, filtx->compute(acc.getIndex(0)));
    acc.setIndex(1, filty->compute(acc.getIndex(1)));

    acc.rotate(-robot.theta);
    // double integration...
    prev_vel = prev_vel.add_by_vect(acc);
    prev_dis = prev_dis.add_by_vect(prev_vel).mult(G);
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
