#include "main.h"
#include "depend.h"
Items items;
Robot robot;
Location map;
Path road;

// Runs initialization code. This occurs as soon as the program is started.
void initialize()
{
	pros::lcd::initialize();
	// initialize objects...
	items.initialize();
	robot.initialize(items);
	map.initialize(robot);
	road.initialize(10, 10);
}

// Runs while the robot is in the disabled state
void disabled() {}

// Runs after initialize(), and before autonomous when connected to the Field Management System or the VEX Competition Switch.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous()
{
	// TODO:
	while (true) {
		if (items.master->get_digital(DIGITAL_A)) return;
		if (items.master->get_digital(DIGITAL_B)) {
			// send to remote:
			items.master->print(0, 0, "(%f, %f) bearing %f [abs: %f]", robot.x, robot.y, robot.theta, robot.get_abs_angle());
		}
		pros::lcd::print(1, "x: %f", robot.x);
		pros::lcd::print(2, "y: %f", robot.y);
		pros::lcd::print(3, "theta: %f", robot.theta);
		pros::lcd::print(4, "abs theta: %f", robot.get_abs_angle());
		pros::lcd::print(5, "press A to exit or B to send to remote...");

		map.update();
		pros::delay(AUTON_LOOP_DELAY);
	}
}

// Runs the operator control code.
void opcontrol()
{
	// pros::lcd::print(1, "starting drive train...");
	// pros::delay(1000);

	bool temp = false;
	bool auton = false;
	// Variables for Smooth Drive:
	#ifdef SMOOTH_CONSTANT
	int speedr = 0; // speed for right
	int speedl = 0; // speed for left
	#endif
	// Driver Code:
	while (true)
	{
		// DRIVE TRAIN ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

		pros::lcd::print(1, "x: %f", robot.x);
		pros::lcd::print(2, "y: %f", robot.y);
		pros::lcd::print(3, "theta: %f", robot.theta);
		pros::lcd::print(4, "abs theta: %f", robot.get_abs_angle());
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

