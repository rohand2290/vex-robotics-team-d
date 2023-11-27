#include "main.h"
#include "depend.h"
Items items;
Robot robot;
Location map;

// Runs initialization code. This occurs as soon as the program is started.
void initialize()
{
	pros::lcd::initialize();
	// initialize objects...
	items.initialize();
	robot.initialize(items);
	map.initialize(robot);
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
	// pros::lcd::print(1, "starting drive train...");
	// pros::delay(1000);

	bool temp = false;
// Variables for Smooth Drive:
#ifdef SMOOTH_CONSTANT
	int speedr = 0; // speed for right
	int speedl = 0; // speed for left
#endif
	// Driver Code:
	while (true)
	{
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

		pros::lcd::print(1, "right: %i", items.encoder_right->get_position());
		pros::lcd::print(2, "left: %i", items.encoder_left->get_position());
		pros::lcd::print(3, "center: %i", items.encoder_center->get_position());
		pros::lcd::print(4, "right: %f in", robot.right_abs_dist());
		pros::lcd::print(5, "left: %f in", robot.left_abs_dist());
		pros::lcd::print(6, "center: %f in", robot.center_abs_dist());

		map.update();

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
	}
}

// PID ===========================================================================================================
double P(double error, bool isturn) { 
	return error * (isturn ? TURN_KP : POWER_KP); 
}
double I(double error, double& integral, Waypoint& goal, bool isturn) {
	integral += error;
	if (goal.x == robot.x && goal.y == robot.y || error == 0) { // TODO: WARNING
		integral = 0;
	}
	double max = (isturn ? TURN_ERROR_MAX : POWER_ERROR_MAX);
	double min = (isturn ? TURN_ERROR_MIN : POWER_ERROR_MIN);
	if (max < error || min > error) {
		integral = 0;
	}
	return integral * (isturn ? TURN_KI : POWER_KI);
}
double D(double& prev_error, double error, bool isturn) {
	return (error - prev_error) * (isturn ? TURN_KD : POWER_KD);
}
double PID(double error, double& integral, double& prev_error, Waypoint& goal, bool isturn) {
	return P(error, isturn) + I(error, integral, goal, isturn) + D(prev_error, error, isturn);
}

