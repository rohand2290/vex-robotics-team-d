#include "main.h"
#include "depend.h"

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
	road.initialize(0, 10);

	// use array to initialize road

}
// Runs while the robot is in the disabled state
void disabled() { items.stop(); }
// Runs after initialize(), and before autonomous when connected to the Field Management System or the VEX Competition Switch.
void competition_initialize() {}

/*
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
*/

// Runs the user autonomous code.
void autonomous()
{
	// pros::lcd::clear();

	// for (int i = 0; i < road.size(); ++i) {
	// 	Waypoint current_goal = road.get_latest();
	// 	current_goal.execute_command(robot);

	// 	for (int j = 0; j < 100 * sqrt(current_goal.x*current_goal.x + current_goal.y*current_goal.y); ++j) {
	// 		std::vector<double> vect = map.updatePID(current_goal);

	// 		robot.set_right_side(vect[0] - vect[1]);
	// 		robot.set_left_side(vect[0] + vect[1]);
			
	// 		pros::lcd::print(0, "x: %f", robot.x);
	// 		pros::lcd::print(1, "y: %f", robot.y);
	// 		pros::lcd::print(2, "theta: %i", robot.theta);
	// 		UPDATE_COORDS();
	// 		pros::delay(AUTON_LOOP_DELAY);
	// 	}
	// 	robot.x = 0;
	// 	robot.y = 0;
	// 	road.pop_latest();
	// }

	items.left1->move_velocity(50); items.left2->move_velocity(50); items.left3->move_velocity(50);
	items.right1->move_velocity(50); items.right2->move_velocity(50); items.right3->move_velocity(50);
	pros::delay(2000);
	items.stop();

	pros::lcd::clear();
	pros::lcd::print(0, "ROUTINE COMPLETE...");
	TERMINATE();
}

// Runs the operator control code.
void opcontrol()
{
	//autonomous();
	items.stop();
	// end of auton:
	items.master->clear();
	items.master->print(0, 0, "GO!"); 

	bool temp = false;
	bool auton = false;
	// Variables for Smooth Drive:
	int speedr = 0; // speed for right
	int speedl = 0; // speed for left
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
				speedl
			);
			// actions acording to buttons:
			robot.set_intake(items.master->get_digital(DIGITAL_L1), items.master->get_digital(DIGITAL_L2));
			robot.set_flywheel(
				items.master->get_digital(DIGITAL_UP),
				items.master->get_digital(DIGITAL_DOWN));

			if (items.master->get_digital_new_press(DIGITAL_Y)) items.initpos = !items.initpos;
			if (items.master->get_digital_new_press(DIGITAL_R1)) items.initpos2 = !items.initpos2;
			items.pto->set_value(items.initpos);
			items.wings->set_value(items.initpos2);

			pros::lcd::print(0, "%i", robot.theta);
			items.master->print(0, 0, "%i", robot.theta);

			UPDATE_COORDS();

			pros::delay(OPCONTROL_LOOP_DELAY);
	}
}

