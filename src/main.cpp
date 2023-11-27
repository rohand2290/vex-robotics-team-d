#include "main.h"
#include "depend.h"
Items items;
Robot robot;

// Runs initialization code. This occurs as soon as the program is started.
void initialize()
{
	pros::lcd::initialize();
	// initialize objects...
	items.initialize();
	robot.initialize(items);
}

// Runs while the robot is in the disabled state
void disabled() {}

// Runs after initialize(), and before autonomous when connected to the Field Management System or the VEX Competition Switch.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous()
{
	// TODO:
	// while (true) {

	// 	robot.update_coords();
	// 	pros::delay(AUTON_LOOP_DELAY);
	// }
}

// Runs the operator control code.
void opcontrol()
{
	pros::lcd::print(1, "starting code...");
	pros::delay(1000);

	double old_l = 0;
	double old_r = 0;
	double old_c = 0;
	double old_t = 0;

	double new_t = 0;

	double rel_l = 0;
	double rel_r = 0;
	double rel_c = 0;
	double rel_t = 0;

	bool temp = false;
// Variables for Smooth Drive:
#ifdef SMOOTH_CONSTANT
	int speedr = 0; // speed for right
	int speedl = 0; // speed for left
#endif
	// Driver Code:
	while (true)
	{
		// {
		// 	rel_l = robot.left_abs_dist() - old_l;
		// 	rel_r = robot.right_abs_dist() - old_r;
		// 	rel_c = robot.center_abs_dist() - old_c;
		// 	new_t = calcThetaOrient(old_t, rel_l, rel_r, old_t, new_t);
		// }

		// set speed chassis
		robot.set_speed_chassis(
			items.master->get_analog(ANALOG_LEFT_Y),
			items.master->get_analog(ANALOG_RIGHT_X),
			__LINE__,
			speedr,
			speedl);
		// actions acording to buttons:
		robot.set_intake(items.master->get_digital(DIGITAL_L1), items.master->get_digital(DIGITAL_L2));
		robot.set_turret(
			items.master->get_digital(DIGITAL_UP),
			items.master->get_digital(DIGITAL_DOWN));
		robot.set_puncher(items.master->get_digital(DIGITAL_A));
		pros::delay(OPCONTROL_LOOP_DELAY);

		if (items.master->get_digital(DIGITAL_Y))
		{
			if (!temp) temp = true;
			else continue;
			items.initpos = !items.initpos;
			items.pto1->set_value(items.initpos);
			items.pto2->set_value(items.initpos);
		}
		if (!items.master->get_digital(DIGITAL_Y)) temp = false;
		
		robot.update_coords(rel_l, rel_r, rel_c);

		pros::lcd::print(1, "right: %i", items.encoder_right->get_position());
		pros::lcd::print(2, "left: %i", items.encoder_left->get_position());
		pros::lcd::print(3, "center: %i", items.encoder_center->get_position());
		pros::lcd::print(4, "right: %i in", (double)items.encoder_right->get_position() / 360000.0);
		pros::lcd::print(5, "left: %i in", (double)items.encoder_left->get_position() / 360000.0);
		pros::lcd::print(6, "center: %i in", (double)items.encoder_center->get_position() / 360000.0);

		// {
		// 	old_l = robot.left_abs_dist();
		// 	old_r = robot.right_abs_dist();
		// 	old_c = robot.center_abs_dist();
		// 	old_t = new_t;
		// }
	}
}

// ODOM =====================================================================================================
// double calcThetaOrient( // calculates all the changes and updates them & finds the change in theta.
// 	double old_t, 
// 	double rel_l, 
// 	double rel_r, 
// 	double rel_c, 
// 	double new_t
// )
// {
// 	MatrixXd mat = local_offset(new_t, old_t, rel_c, rel_r);
// 	double avgOr = avgOrient(old_t, new_t);
// 	MatrixXd change = globalOffset(old_t, new_t, mat);

// 	robot.x += change[0];
// 	robot.y += change[1];

// 	return old_t + (rel_l - rel_r) / (ROBOT_WIDTH);
// }
// MatrixXd local_offset(double new_t, double old_t, double rel_c, double rel_r)
// {
// 	MatrixXd m(2, 1);
// 	if (new_t - old_t == 0)
// 	{
// 		m(0, 0) = rel_c;
// 		m(0, 1) = rel_r;
// 	}
// 	else
// 	{
// 		m(0, 0) = rel_c / (new_t - old_t) + 9.251969;
// 		m(0, 1) = rel_r / (new_t - old_t) + (ROBOT_WIDTH / 2);
// 		m *= 2 * sin(old_t / 2);
// 	}
// 	return m;
// }
// double avgOrient(double old_t, double new_t) {
// 	return old_t + (new_t - old_t) / 2;
// }
// MatrixXd globalOffset(double old_t, double new_t, MatrixXd delta_dl) {
// 	double r = sqrt(robot.x*robot.x + robot.y*robot.y);
// 	double theta = atan2(robot.y, robot.x) - avgOrient(old_t, new_t);

// 	double x = r * cos(theta);
// 	double y = r * sin(theta);

// 	MatrixXd mat(x, y);
// 	return mat;
// }

// PID ===========================================================================================================
// double P(double error) { return error * KP; }
// double I(double error, double& integral, Waypoint& goal) {
// 	integral += error;
// 	if (goal.x == robot.x && goal.y == robot.y || error == 0) { // TODO: WARNING
// 		integral = 0;
// 	}
// 	if (ERROR_MAX < error || ERROR_MIN > error) {
// 		integral = 0;
// 	}
// 	return integral * KI;
// }
// double D(double& prev_error, double error) {
// 	return (error - prev_error) * KD;
// }
// double PIDPower(double error, double& integral, double& prev_error, Waypoint& goal) {
// 	return P(error) + I(error, integral, goal) + D(prev_error, error);
// }

