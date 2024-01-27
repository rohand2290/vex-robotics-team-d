#include "depend.h"
#include "location.h"

double Location::right_abs_dist()
{ // in terms of inches
    Items& items = robot->items;
    double normal = items.right1->get_position() + items.right2->get_position() + items.right3->get_position();
    return (normal / 3) * WHEEL_C;
}

double Location::left_abs_dist()
{ // in terms of inches
    Items& items = robot->items;
    double normal = items.left1->get_position() + items.left2->get_position() + items.left3->get_position();
    return (normal / 3) * WHEEL_C;
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

    dis.initialize(POWER_KP, POWER_KI, POWER_KD);
    turn_casual.initialize(TURN_KP, TURN_KI, TURN_KD);
    swing.initialize(SWING_KP, SWING_KI, SWING_KD);
}

std::vector<double> Location::update() {
    rel_l = left_abs_dist() - old_l;
	rel_r = right_abs_dist() - old_r;
    // rel_th = robot->get_abs_angle() - old_th;
    robot->theta = robot->get_abs_angle();

    double time_factor = MECH_ADVANTAGE * 1.2;
    double mag = (rel_l + rel_r) / 2;
    std::vector<double> arr = {
        (mag * sin(robot->degrees_to_radians(robot->theta))) * time_factor,
        (mag * cos(robot->degrees_to_radians(robot->theta))) * time_factor,
    };

    cx += arr[0];
    cy += arr[1];

    old_l = left_abs_dist();
	old_r = right_abs_dist();
    // old_th = robot->get_abs_angle();

    return {cx, cy};
}

static double angleDifference(double start, double end) {
    double red = (end-start);
    if (red > 180) {
        return red - 360;
    }
    return red;
}

static double toTheta(double x, double y) {
    if (x == 0) x += 0.00000001;
    double a = atan(abs(y) / abs(x));

    if (x >= 0) {
        if (y >= 0) return a;
        else return 2 * PI - a;
    } else {
        if (y >= 0) return PI - a;
        else return PI + a;
    }
}

std::vector<double> Location::updatePID(Waypoint& goal, CartesianLine& robot_line, CartesianLine& goal_line, bool turn) {
    if (goal.command == "move") {
        error_turn_casual = angleDifference(robot->items.imu->get_rotation(), old_angle);
        double turn = turn_casual.update(error_turn_casual);
        
        double val = goal.param1 < 0 ? -sqrt(cx*cx + cy*cy) : sqrt(cx*cx + cy*cy);
        error = goal.param1 - val;
        if (robot->items.master->get_digital(pros::E_CONTROLLER_DIGITAL_A))
            robot->items.master->print(0, 0, "%f", error);
        double power = dis.update(error);
        abs_timer++;

        if (power > 127) power = 127;
        else if (power < -127) power = -127;
        // power *= abs(cos(error_turn_casual)); // uncomment to enable angle stabalization:

        std::vector<double> v = {power, power}; // add -turn, turn to enable angle stabalization.

        if (abs(error) < MIN_ALLOWED_ERROR) {
            if (turn) timer = MIN_ALLOWED_ERROR_TIME;
            timer++;
        }
        return v;

    } else if (goal.command == "turn") {
        error_turn_casual = angleDifference(robot->items.imu->get_rotation(), goal.param1 + old_angle);
        
        double turn = turn_casual.update(error_turn_casual);

        std::vector<double> v = {-turn, turn};

        if (turn && error <= 0) timer = MIN_ALLOWED_ERROR_TIME; 
        else if (abs(error_turn_casual) < MIN_ALLOWED_ERROR_DEG && !turn) timer++;

        abs_timer++;
        return v;
    } else if (goal.command == "power") {
        is_bashing = true;
        robot->set_both_sides(goal.param1, goal.param1);
        pros::delay(goal.param2);
        robot->break_absolute();
        return {0, 0};
    } else if (goal.command == "curve") { //// @TODO
        // // curve PID: (first val is mag, second is ending theta)
        // double x = sin(robot->degrees_to_radians(goal.param2)) * goal.param1;
        // double y = cos(robot->degrees_to_radians(goal.param2)) * goal.param1;
        // double t = robot->theta - robot->radians_to_degrees(atan(x / y));
        
        // CartesianCircle temp(0, 0, 0);
        // double arclen = (goal.param2 * PI * goal.param1) / (360.0 * sin(t / 2));
        // double radius = temp.compute_radius(x, y, t);
        // std::vector<Point2D> points = temp.find_intersections(x, y, radius);

        // // construct circles to follow:
        // CartesianCircle c(0, 0, 0);
        // {
        //     // FIX:
        //     CartesianCircle c1(points[0].x, points[0].y, radius);
        //     CartesianCircle c2(points[1].x, points[1].y, radius);
        //     TERMINATE();
        //     if (
        //         ABS(standrad_to_bearing(normalize(c1.derivative_at(0))) - robot->theta) >
        //         ABS(standrad_to_bearing(normalize(c2.derivative_at(0))) - robot->theta)
        //     ) c = c2;
        //     else c = c1;
        // }        
    
        // // align bot at init:
        // double init_angle = c.derivative_at(0);
        // long ti = 0;
        // while (ti < MIN_ALLOWED_ERROR_TIME) {
        //     // turn PID:
        //     error_l = angleDifference(robot->theta, init_angle);
        //     double turn = pid(error_l, integral_l, prev_error_l, goal, true);
        //     pros::lcd::print(1, "%f", turn);
        //     std::vector<double> v = {-turn, turn};
        //     if (ABS(error_l) < MIN_ALLOWED_ERROR_DEG) ti++;
        // }
        // // start follwing path:
        // ti = 0;
        // pros::lcd::print(1, "DONE!");
        // TERMINATE();
        // while (ti < MIN_ALLOWED_ERROR_TIME) {
        //     double error_x = goal_line.x - robot->x;
        //     double error_y = goal_line.y - robot->y;
        //     robot_line.slope = tan(robot->degrees_to_radians(robot->theta));
        //     goal_line.slope = robot_line.get_perp(robot_line.get_slope());
        //     int ci = 1;
        //     if (error_y < 0) ci = -1;
        //     error = ci * sqrt(error_x*error_x + error_y*error_y);
        //     double power = pid(error, integral, prev_error, goal, false);

        //     error_l = angleDifference(robot->theta, c.derivative_at(robot->x));
        //     double turn = pid(error_l, integral_l, prev_error_l, goal, true);

        //     std::vector<double> v = {power - turn, power + turn};
        //     robot->set_both_sides(v[1], v[0]);

        //     if (error < MIN_ALLOWED_ERROR) ti++;
        // }
    } else if (goal.command == "swing") {
        error_turn_casual = angleDifference(robot->items.imu->get_rotation(), goal.param1);
        double turn = swing.update(error);
        std::vector<double> v;
        if (goal.param1 < 0) v = {-turn, 0};
        else v = {0, turn};

        if (abs(error_turn_casual) < MIN_ALLOWED_ERROR_DEG) timer++;
        abs_timer++;
        return v;
    } else if (goal.command == "raw") { //// @TODO
        double dx = goal.param1 - cx;
        double dy = goal.param2 - cy;
        error_turn_casual = angleDifference(robot->items.imu->get_rotation(), toTheta(dx, dy));
        double turn = turn_casual.update(error_turn_casual);
        
        double val = sqrt(dx*dx + dy*dy);
        if (dx < 0) {
            if (cx < goal.param1 || cy > goal.param2) val *= -1;
        } else {
            if (cx > goal.param1 || cy > goal.param2) val *= -1;
        }

        if (robot->items.master->get_digital(pros::E_CONTROLLER_DIGITAL_A))
            robot->items.master->print(0, 0, "%f", error);
        double power = dis.update(error);
        abs_timer++;

        if (power > 127) power = 127;
        else if (power < -127) power = -127;
        power *= abs(cos(error_turn_casual));

        std::vector<double> v = {power - turn, power + turn};
        if (abs(error) < MIN_ALLOWED_ERROR) {
            if (turn) timer = MIN_ALLOWED_ERROR_TIME;
            timer++;
        }
        return v;
    }
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
    dis.reset();
    turn_casual.reset();
    swing.reset();

    timer = 0;
    abs_timer = 0;
    cx = 0;
    cy = 0;

    old_l = 0;
    old_r = 0;
    rel_r = 0;
    rel_l = 0;

    error = 0;
    error_turn_casual = 0;
    error_swing = 0;

    is_bashing = 0;
}

bool Location::is_running() {
    // if (abs_timer >= MIN_ALLOWED_ERROR_TIMEOUT) return false;
    if (is_bashing) return false;
    return timer <= MIN_ALLOWED_ERROR_TIME && abs_timer <= MIN_ALLOWED_ERROR_TIMEOUT;
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
