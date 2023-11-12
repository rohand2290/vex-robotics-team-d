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
	double integral_r = 0;
	double integral_l = 0;
	double integral_c = 0;
	int speedr = 0;
	int speedl = 0;

	while (true) {
		integral_r += robot.get_error_r() * AUTON_LOOP_DELAY;

		pros::delay(AUTON_LOOP_DELAY);
	}
}

// Runs the operator control code.
void opcontrol() {
	pros::lcd::print(1, "starting code...");
	pros::delay(1000);

	bool temp = false;
	// Variables for Smooth Drive:
	#ifdef SMOOTH_CONSTANT
	int speedr = 0; // speed for right
	int speedl = 0; // speed for left
	#endif
	// Driver Code:
	while (true) {
		// set speed chassis
		robot.set_speed_chassis(
			items.master->get_analog(ANALOG_LEFT_Y),
			items.master->get_analog(ANALOG_RIGHT_X),
			__LINE__,
			&speedr,
			&speedl
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
	}
}
