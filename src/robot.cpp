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
