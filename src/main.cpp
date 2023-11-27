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
		pros::lcd::print(4, "right: %i in", (double)items.encoder_right->get_position() / 360000.0);
		pros::lcd::print(5, "left: %i in", (double)items.encoder_left->get_position() / 360000.0);
		pros::lcd::print(6, "center: %i in", (double)items.encoder_center->get_position() / 360000.0);

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

