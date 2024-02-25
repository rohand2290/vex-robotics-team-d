#include "depend.h"
#include "location.h"

double Location::right_abs_dist()
{ // in terms of inches
    Items& items = robot->items;
    double normal = items.right1->get_position() + items.right2->get_position() + items.right3->get_position();
    return (normal / 3) * WHEEL_C * MECH_ADVANTAGE;
}

double Location::left_abs_dist()
{ // in terms of inches
    Items& items = robot->items;
    double normal = items.left1->get_position() + items.left2->get_position() + items.left3->get_position();
    return (normal / 3) * WHEEL_C * MECH_ADVANTAGE;
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
    rel_th = robot->get_abs_angle() - old_th;
    robot->theta = robot->get_abs_angle(true);

    double mag = (rel_l + rel_r) / 2;
    std::vector<double> arr = {
        mag * sin(rel_th),
        mag * cos(rel_th),
    };

    cx += arr[0];
    cy += arr[1];

    old_l = left_abs_dist();
	old_r = right_abs_dist();
    old_th = robot->get_abs_angle();

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
        power *= abs(cos(robot->degrees_to_radians(error_turn_casual))); // uncomment to enable angle stabalization:

        std::vector<double> v = {power - turn, power + turn}; // add -turn, turn to enable angle stabalization.

        if (abs(error) < MIN_ALLOWED_ERROR) {
            if (turn) timer = MIN_ALLOWED_ERROR_TIME;
            timer++;
        }
        return v;

    } else if (goal.command == "turn") {
        is_turning = true;
        if (goal.param2 == 1)
            error_turn_casual = angleDifference(robot->items.imu->get_rotation(), goal.param1);
        else
            error_turn_casual = angleDifference(robot->items.imu->get_rotation(), goal.param1);

        double turn = turn_casual.update(error_turn_casual);

        std::vector<double> v = {-turn, turn};

        if (turn && error <= 0) timer = MIN_ALLOWED_ERROR_TIME; 
        else if (abs(error_turn_casual) < MIN_ALLOWED_ERROR_DEG && !turn) timer++;

        if (robot->items.master->get_digital(pros::E_CONTROLLER_DIGITAL_A))
            robot->items.master->print(0, 0, "%f", error);

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
    } else if (goal.command == "raw") {
        double dx = goal.param1 - cx;
        double dy = goal.param2 - cy;
        error_turn_casual = angleDifference(robot->items.imu->get_rotation(), standrad_to_bearing(toTheta(dx, dy)));
        double turn = turn_casual.update(error_turn_casual);
        
        error = sqrt(dx*dx + dy*dy);
        // if (dx < 0) {
        //     if (cx < goal.param1 || cy > goal.param2) error *= -1;
        // } else {
        //     if (cx > goal.param1 || cy > goal.param2) error *= -1;
        // }
        if (robot->items.master->get_digital(pros::E_CONTROLLER_DIGITAL_A))
            robot->items.master->print(0, 0, "%f", error);
        double power = dis.update(error);
        abs_timer++;

        if (power > 127) power = 127;
        else if (power < -127) power = -127;
        power *= abs(COS_FACTOR * cos(SENSITIVE_FACTOR * robot->degrees_to_radians((int)error_turn_casual)));

        std::vector<double> v = {power - turn, power + turn};
        if (abs(error) < MIN_ALLOWED_ERROR) {
            if (turn) timer = MIN_ALLOWED_ERROR_TIME;
            timer++;
        }
        return v;
        // return {0, 0};
    }
    return {0, 0};
}

void Location::reset_all()
{
    robot->items.left2->tare_position();
	robot->items.right2->tare_position();
    robot->items.left1->tare_position();
	robot->items.right1->tare_position();
    robot->items.left3->tare_position();
	robot->items.right3->tare_position();

    old_angle += robot->items.imu->get_rotation();
    robot->items.imu->tare_rotation();
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
    is_turning = 0;
}

bool Location::is_running() {
    // if (abs_timer >= MIN_ALLOWED_ERROR_TIMEOUT) return false;
    if (is_bashing) return false;
    return timer <= MIN_ALLOWED_ERROR_TIME && abs_timer <= (is_turning ? 
        MIN_ALLOWED_ERROR_TIMEOUT_DEG : MIN_ALLOWED_ERROR_TIMEOUT);
}

double Location::get_angle_abs() {
    return robot->items.imu->get_rotation() - old_angle;
}

Autotuner::Autotuner(Location& l, Robot& r): maping(l), robot(r) {}

static double get_avg_error(std::vector<double> model, std::vector<double> depict) {
    double ret;
    int i = 0;
    for (; i < model.size(); ++i) {
        ret += model[i] - depict[i];
    }
    return ret / i;
}
static double get_avg_error(std::vector<double> model, std::vector<double> depict, std::vector<double> add) {
    double ret;
    int i = 0;
    for (; i < model.size(); ++i) {
        ret += model[i] - (depict[i] + add[i]);
    }
    return ret / i;
}
/**
 * @brief Prerequisites: Odom is Tuned, and KP = 1; KI = 0; KD = 0;
 * 
 * @param command sample command to follow
 * @param time amt of time to take for movement
 */
constexpr int MAX_ITERATIONS = 100;
void Autotuner::run(Waypoint command, int time) {
    std::vector<double> proportional;
    std::vector<double> integral;
    std::vector<double> derivative;
    std::vector<double> model;

	// step 1: get raw data.
	maping.reset_all();
	robot.items.autonmous = true;
	int count = 0;
	bool error_type = false;
	maping.old_angle = robot.items.imu->get_rotation();
	error_type = command.execute_aux_command(&robot);
	CartesianLine robot_line(0, robot.x, robot.y);
	CartesianLine goal_line(0, command.param1 * sin(robot.theta), command.param1 * cos(robot.theta));
    do
	{
		maping.start_iter = pros::millis();
		std::vector<double> vect;
		if (command.is_motion_command()) vect = maping.updatePID(command, robot_line, goal_line, error_type);
		else break;
		robot.set_both_sides(vect[1], vect[0]);
		maping.update();

        if (command.command == "move") {
            proportional.push_back(maping.error);
            integral.push_back(maping.dis.integral);
            derivative.push_back(maping.error - maping.dis.prev_error);
        } else if (command.command == "turn") {
            proportional.push_back(maping.error_turn_casual);
            integral.push_back(maping.turn_casual.integral);
            derivative.push_back(maping.error_turn_casual - maping.turn_casual.prev_error);
        }

        if (count >= time) model.push_back(command.param1);
        else model.push_back(0);

        pros::delay(1);
        count++;
	} while (maping.is_running());
	robot.set_both_sides(0, 0);
    robot.items.stop();
	maping.cx = 0;
	maping.cy = 0;
	maping.reset_all();

    model.shrink_to_fit();
    proportional.shrink_to_fit();
    integral.shrink_to_fit();
    derivative.shrink_to_fit();

    pros::lcd::clear();
    pros::lcd::print(0, "Calculating Constants...");

    // step 2: start tuning...
    // step 2.1: tune kp

    std::vector<double> p = proportional;
    double best_error = 99999999;
    double nKP = 1;
    for (int _ = 0; _ < MAX_ITERATIONS; ++_) {
        double diff = 0;
        int i = 0;
        for (; i < model.size(); ++i) {
            diff += proportional[i] / p[i];
        }
        diff /= i;
        for (int i = 0; i < p.size(); ++i) {
            p[i] *= diff;
        }

        double error = get_avg_error(model, p);
        if (error < best_error) { 
            nKP = diff;
            best_error = error; 
        }
    }

    std::vector<double> d = derivative;
    best_error = 99999999;
    double nKD = 1;
    for (int _ = 0; _ < MAX_ITERATIONS; ++_) {
        double diff = 0;
        int i = 0;
        for (; i < model.size(); ++i) {
            diff += proportional[i] / (p[i] + d[i]);
        }
        diff /= i;
        for (int i = 0; i < d.size(); ++i) {
            d[i] *= diff;
        }

        double error = get_avg_error(model, p, d);
        if (error < best_error) { 
            nKD = diff;
            best_error = error; 
        }
    }

    // step 3 print out output...
    pros::lcd::clear();
    pros::lcd::print(0, "PID Values:");
    pros::lcd::print(2, "KP: %f", nKP);
    pros::lcd::print(3, "KI: Just do some small number...");
    pros::lcd::print(4, "KD: %f", nKD);
    // step 4 terminate...
    robot.items.stop();
    TERMINATE();
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
