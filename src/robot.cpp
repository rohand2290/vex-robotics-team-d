#include "depend.h"
#include "robot.h"

double Robot::right_abs_dist() { // in terms of inches
    return WHEEL_C * (items.encoder_right->get_position() / 36000);
}

double Robot::left_abs_dist() { // in terms of inches
    return WHEEL_C * (items.encoder_left->get_position() / 36000);
}

double Robot::center_abs_dist() { // in terms of inches
    return WHEEL_C * (items.encoder_center->get_position() / 36000);
}

// desired is in inches
double Robot::get_error_r(double desired) { return desired - right_abs_dist(); }
// desired is in inches
double Robot::get_error_l(double desired) { return desired - left_abs_dist(); }
// desired is in inches
double Robot::get_error_c(double desired) { return desired - center_abs_dist(); }

void Robot::initialize(Items &i)
{
    items = i;
    x = 0;
    y = 0;
    theta = 0;
}

Robot::~Robot() {}

void Robot::set_right_side(int analog) {
    items.left1->move(analog); 
    items.left2->move(analog); 
    items.left3->move(analog);
}

void Robot::set_left_side(int analog) {
    items.right1->move(analog); 
    items.right2->move(analog); 
    items.right3->move(analog);
}

// here, y represents power and x represents turn, since this is arcade drive style.
void Robot::set_speed_chassis(int y, int x, long long line, int* speedr = nullptr, int* speedl = nullptr)
{
	int left = (y + (x * TURN_PERCENT)) * MOTOR_PERCENT;
	int right = (y - (x * TURN_PERCENT)) * MOTOR_PERCENT;
	// set the wheels...
	#ifdef SMOOTH_CONSTANT
    if (speedl == nullptr || speedr == nullptr) {
        // this should not happen...
        pros::lcd::print(1, "ERROR: The chassis speed function has not been setup correctly."
        " Look at line %i", line);
        while (true);
    }
    // left side:
	if (*speedl >= left) speedl -= SMOOTH_CONSTANT;
	else if (*speedl < left) speedl += SMOOTH_CONSTANT;
	// right side:
	if (*speedr >= right) speedr -= SMOOTH_CONSTANT;
	else if (*speedr < right) speedr += SMOOTH_CONSTANT;

	set_left_side(*speedl);
	set_right_side(*speedr);
	#else
	set_left_side(left);
	set_right_side(right);
	#endif
}

void Robot::set_intake(int analog)  {// true means in, false means out
    if (analog) {
        items.intake_left->move(INTAKE_IN_SPEED);
		items.intake_right->move(INTAKE_IN_SPEED);
    } else {
        items.intake_left->move(INTAKE_OUT_SPEED);
		items.intake_right->move(INTAKE_OUT_SPEED);
    }
}

void Robot::set_turret(int up, int down) {
    if (up) {
		items.turret->move(TURRET_SPEED);
	} else {
		items.turret->move(0);
	}
	if (down) {
		items.turret->move(-TURRET_SPEED);
	} else {
		items.turret->move(0);
	}
}

void Robot::set_puncher(int analog) {
    items.puncher1->set_value(analog);
	items.puncher2->set_value(analog);
}

double Robot::radians_to_degrees(double radians) {
    return (radians * 180) / PI;
}

double Robot::degrees_to_radians(double degrees) {
    return (degrees * PI) / 180;
}

void Robot::update_coords ()
{
    double r_and_l_speed = items.encoder_right->get_velocity() - items.encoder_left->get_velocity(); // centidegrees per sec
    double c_speed = items.encoder_center->get_velocity();
    // calculate theta:
    /** ============== LOGIC ====================
    * just r & l: change in theta = (360000 * right speed - left speed)/(2 * pi * width of robot)
    * just c: change in theta = (360000 * center speed) / (2 * pi * width of robot)
    * if change with c != change with r & l:
    *   some outer force pushed the bot.
    *   do something to accomadate for this.
    */
    double by_rl = (360000 * r_and_l_speed) / (2 * PI * ROBOT_WIDTH);
    double by_c = (360000 * c_speed) / (2 * PI * ROBOT_WIDTH);
    if (by_rl - by_c > ODOM_ACCURACY && by_c - by_rl > ODOM_ACCURACY && CHECK_FOR_ENV_FORCES) {
        env_push_error();
    }
    theta += by_c;
    // calculate distance:
    /** ============== LOGIC =====================
     * t = right speed or left speed depending on which is less
     * vertical translation = sin(theta / 100) * t
     * horizontal translation = cos(theta / 100) * t
    */
    double t = items.encoder_right->get_velocity() > items.encoder_left->get_velocity() ? 
            items.encoder_left->get_velocity() : items.encoder_right->get_velocity();
    y += sin(theta + 90) * t; // plus 90 since this is an example of bearing applications.
    x += cos(theta + 90) * t;
}

// this function is called when an unusual outside force is detected.
static void env_push_error() {
    pros::lcd::print(1, "There was an unwanted push on the bot that has been detected. This is part of the auton routine to ensure everything is working.");
    while (true);
}
