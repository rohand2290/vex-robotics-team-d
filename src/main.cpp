#include "main.h"
#include "depend.h"

Items items;
Robot robot;

// Runs initialization code. This occurs as soon as the program is started.
void initialize() {
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
void autonomous() {
	// TODO: 
	// while (true) {
		

	// 	robot.update_coords();
	// 	pros::delay(AUTON_LOOP_DELAY);
	// }
}

// Runs the operator control code.
void opcontrol() {
	pros::lcd::print(1, "starting code...");
	pros::delay(1000);

	double old_l = 0;
	double old_r = 0;
	double old_c = 0;

	double rel_l = 0;
	double rel_r = 0;
	double rel_c = 0;

	bool temp = false;
	// Variables for Smooth Drive:
	#ifdef SMOOTH_CONSTANT
	int speedr = 0; // speed for right
	int speedl = 0; // speed for left
	#endif
	// Driver Code:
	while (true) {
		rel_l = robot.left_abs_dist() - old_l;
		rel_r = robot.right_abs_dist() - old_r;
		rel_c = robot.center_abs_dist() - old_c;

		// set speed chassis
		robot.set_speed_chassis(
			items.master->get_analog(ANALOG_LEFT_Y),
			items.master->get_analog(ANALOG_RIGHT_X),
			__LINE__,
			speedr,
			speedl
		);
		// actions acording to buttons:
		robot.set_intake(items.master->get_digital(DIGITAL_L1));
		robot.set_turret(
			items.master->get_digital(DIGITAL_UP), 
			items.master->get_digital(DIGITAL_DOWN)
		);
		robot.set_puncher(items.master->get_digital(DIGITAL_A));
		pros::delay(OPCONTROL_LOOP_DELAY);

		if (items.master->get_digital(DIGITAL_Y)) {
			if (!temp) temp = true;
			else continue;			
			items.initpos = !items.initpos;
			items.pto1->set_value(items.initpos);
			items.pto2->set_value(items.initpos);
		} if (!items.master->get_digital(DIGITAL_Y)) temp = false;
		robot.update_coords(rel_l, rel_r, rel_c);

		pros::lcd::print(1, "%f", robot.x);
		pros::lcd::print(2, "%f", robot.y);
		pros::lcd::print(3, "%f", robot.theta);

		old_l = robot.left_abs_dist();
		old_r = robot.right_abs_dist();
		old_c = robot.center_abs_dist();
	}
}
