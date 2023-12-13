#include "main.h"
#include "depend.h"

#define UPDATE_COORDS() {\
			std::vector<double> vect = map.update();\
			robot.x += vect[0];\
			robot.y += vect[1]; }

const std::vector<Waypoint> spawn1 = {
	{27, 32.17},
	{42, 0},
	{0, 26.2, "WINGS"},
	{35, 0},
	{0, 27},
	{16.97, 16.97},
	{24, 0, "INTAKE"},
	{52, 0, "MATCH_LOAD"},
	{0, 30},
	{28.99, 28.99},
	{24, 0},
};

const std::vector<Waypoint> spawn2 = { // TODO
	{27, 32.17},
	{42, 0},
	{0, 26.2, "WINGS"},
	{35, 0},
	{0, 27},
	{16.97, 16.97},
	{24, 0, "INTAKE"},
	{52, 0, "MATCH_LOAD"},
	{0, 30},
	{28.99, 28.99},
	{24, 0},
};

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
	// use array to initialize road
	//road.initialize(spawn1);
	road.initialize(0, 5);

}

// Runs while the robot is in the disabled state
void disabled() { items.stop(); }
// Runs after initialize(), and before autonomous when connected to the Field Management System or the VEX Competition Switch.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous()
{
	pros::lcd::clear();

	for (int i = 0; i < road.size(); ++i) {
		Waypoint current_goal = road.get_latest();
		current_goal.execute_command(robot);

		for (int i = 0; i < 1000; ++i) {
			std::vector<double> vect = map.updatePID(current_goal);
			robot.set_both_sides(vect[1], vect[0]);
			
			pros::lcd::print(0, "x: %f", robot.x);
			pros::lcd::print(1, "y: %f", robot.y);
			pros::lcd::print(2, "theta: %f", robot.theta);
			UPDATE_COORDS();
			pros::delay(AUTON_LOOP_DELAY);
		}

		robot.x = 0;
		robot.y = 0;
		road.pop_latest();
	}

	items.master->print(0, 0, "F");
	items.stop();
	TERMINATE();
}

// Runs the operator control code.
void opcontrol()
{
	autonomous();
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
		robot.set_intake(
			items.master->get_digital(DIGITAL_L1), items.master->get_digital(DIGITAL_L2),
			items.master->get_digital_new_press(DIGITAL_A)
		);
		robot.set_flywheel(items.master->get_digital_new_press(DIGITAL_UP));
		robot.set_wings(items.master->get_digital_new_press(DIGITAL_R1));
		robot.set_pto(items.master->get_digital(DIGITAL_Y));

		pros::lcd::print(0, "%f", robot.theta);
		pros::lcd::print(1, "%f", robot.x);
		pros::lcd::print(2, "%f", robot.y);

		UPDATE_COORDS();

		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}

