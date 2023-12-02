#include "main.h"
#include "depend.h"

#ifdef START_RED_ALLY
std::vector<std::vector<int>> roadway = {
	{8, 8},
};
#else
std::vector<int[]> roadway = {
	
};
#endif


#define UPDATE_COORDS() {\
			std::vector<double> vect = map.update();\
			robot.theta = map.normalize(robot.get_abs_angle());\
			robot.x += vect[0];\
			robot.y += vect[1]; }

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
void competition_initialize() { 
	pros::lcd::clear(); 
	items.master->clear();
	items.master->print(0, 0, "Auton..."); 
}

// Test of abs theta measurement:
static void test_angle_odom() {
	pros::lcd::clear();
	pros::lcd::print(0, "press A to test angle odom...");
	while (!items.master->get_digital(DIGITAL_A)) pros::delay(20);

	while (robot.theta < 180) {
		UPDATE_COORDS();
		robot.set_left_side(-60);
		robot.set_right_side(60);
		items.master->print(0, 0, "(%i,%i)b%i", (int)robot.x, (int)robot.y, (int)robot.theta);
	}

	robot.set_left_side(0);
	robot.set_right_side(0);

	pros::lcd::print(0, "TEST COMPLETE. Check for inaccuracies...");
	TERMINATE();
}

// Runs the user autonomous code.
void autonomous()
{
	pros::lcd::clear();
	pros::lcd::print(0, "press A to start PID test point:");
	pros::lcd::print(1, "(%f, %f)", road.get_latest().x, road.get_latest().y);
	while (!items.master->get_digital(DIGITAL_A)) pros::delay(AUTON_LOOP_DELAY);

	for (int i = 0; i < roadway.size(); ++i) {
		Waypoint current_goal = road.get_latest();
		while (!road.goal_reached(current_goal, robot.x, robot.y)) {
			if (items.master->get_digital(DIGITAL_X)) break;

			std::vector<double> vect = map.updatePID(current_goal);

			robot.set_right_side(vect[0] + vect[1]);
			robot.set_left_side(vect[0] - vect[1]);
			
			pros::lcd::print(0, "x pow: %f", robot.x);
			pros::lcd::print(1, "y pow: %f", robot.y);
			pros::lcd::print(2, "theta: %f", robot.theta);
			pros::lcd::print(3, "press X to exit or B to send to remote...");
			items.master->print(0, 0, "%f", robot.y);
			UPDATE_COORDS();
			pros::delay(AUTON_LOOP_DELAY);
		}
		// reset x & y
		// delete current goal
	}
}

// Runs the operator control code.
void opcontrol()
{
	// end of auton:
	items.master->clear();
	items.master->print(0, 0, "GO!"); 
	// UNCOMMENT FOLLOWING TO STOP AUTON TESTING:
	autonomous();

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

			UPDATE_COORDS();
			pros::lcd::print(0, "x: %f", robot.x);
			pros::lcd::print(1, "y: %f", robot.y);
			pros::lcd::print(2, "abs theta: %i", robot.theta);
			items.master->print(0, 0, "(%i,%i)b%i", (int)robot.x, (int)robot.y, (int)robot.theta);

			if (items.master->get_digital_new_press(DIGITAL_Y)) items.initpos = !items.initpos;
			items.pto->set_value(items.initpos);
			items.wings->set_value(items.master->get_digital(DIGITAL_R1));
		}

		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}

