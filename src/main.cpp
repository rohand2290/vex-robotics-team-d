#include "main.h"
#include "depend.h"

struct Instruction {
	double move;
	bool is_turn;
	std::string command;
};

#ifdef START_RED_ALLY
std::vector<Instruction> roadway = {
	{8, 0, ""},
	{-60, 1, ""},
	{28.8, 0, ""},
	{0, 0, "OUT"},
	{-28.8, 0, ""},
	{-130, 1, ""},
	{12.8, 0, "IN"},
	{-125, 1, ""},
	{19, 0, ""},
	{-25, 1, ""},
	{26, 0, ""},
	{45, 0, ""}
};
#else
std::vector<Instruction> roadway = {
	{8, 0, ""},
	{90, 1, ""},
	{20.8, 0, ""},
	{0, 0, "OUT"},
	{-20.8, 0},
	{90, 1, ""},
	{12.8, 0, "IN"},
	{-12.8, 0, ""},
	{110, 1, ""},
	{19, 0, ""},
	{40, 1, ""},
	{26, 0, ""},
	{45, 1, ""},
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
void disabled() { items.stop(); }

// Runs after initialize(), and before autonomous when connected to the Field Management System or the VEX Competition Switch.
void competition_initialize() {}

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
	// UNCOMMENT WHEN TESTING:
	// pros::lcd::print(0, "press A to start PID test point:");
	// pros::lcd::print(1, "(%f, %f)", road.get_latest().x, road.get_latest().y);
	// while (!items.master->get_digital(DIGITAL_A)) pros::delay(AUTON_LOOP_DELAY);

	for (Instruction ins : roadway) {
		if (ins.command == "IN") {
			items.intake_left->move(-255);
			items.intake_right->move(-255);
		} else if (ins.command == "OUT") {
			items.intake_left->move(255);
			items.intake_right->move(255);
		}

		double need_theta = robot.theta;
		double goal_x = robot.x;
		double goal_y = robot.y;

		if (ins.is_turn) {
			need_theta = robot.theta + ins.move;
		} else {
			goal_x = sin(robot.theta) * ins.move;
			goal_y = cos(robot.theta) * ins.move;
		}


		Waypoint current_goal = {goal_x, goal_y};

		while (true) {
			if (items.master->get_digital(DIGITAL_X)) break;

			std::vector<double> vect = map.updatePID(current_goal);

			robot.set_right_side(vect[0] + vect[1]);
			robot.set_left_side(vect[0] - vect[1]);
			
			pros::lcd::print(0, "x goal: %f", goal_x);
			pros::lcd::print(1, "y goal: %f", goal_y);
			pros::lcd::print(2, "theta goal: %f", need_theta);
			pros::lcd::print(3, "press X to exit or B to send to remote...");
			items.master->print(0, 0, "%f", robot.y);
			UPDATE_COORDS();
			pros::delay(AUTON_LOOP_DELAY);
		}
		robot.x = 0;
		robot.y = 0;
		// road.pop_latest();
	}
}

// Runs the operator control code.
void opcontrol()
{
	items.stop();
	// end of auton:
	items.master->clear();
	items.master->print(0, 0, "GO!"); 

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

			if (items.master->get_digital_new_press(DIGITAL_Y)) items.initpos = !items.initpos;
			items.pto->set_value(items.initpos);
			items.wings->set_value(items.master->get_digital(DIGITAL_R1));
		}

		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}

