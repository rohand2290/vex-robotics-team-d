#include "main.h"
#include "depend.h"
#include "tests.h"
using namespace std::chrono;

std::vector<Waypoint> spawn1 = {
#ifndef RIGHT_SIDE
	//WINPOINT LEFT:
	{"move", 15},
	{"wings"},
	{"wait", 500},
	{"turn", 45},
	{"wings"},
	{"move", 20},
	{"turn", 20},
	{"power", 127, 1000},
	{"move", -15},
	{"turn", 90},
	{"move", 50},
	{"move", -26.5},
	{"turn", -60},
	{"move", -47},
#else
	// WINPOINT RIGHT:
	{"move", 15},
	{"wings"},
	{"wait", 500},
	{"turn", -45},
	{"wings"},
	{"move", 20},
	{"turn", -20},
	{"power", 127, 1000},
	{"move", -15},
	{"turn", -90},
	{"move", 50},
	{"move", -26.5},
	{"turn", 60},
	{"move", -47},
#endif
	// // 5 ball AUTON:
	// {"in"},
	// {"move", 60},
	// {"turn", -45},
	// {"move", -29},
	// {"move", 2},
	// {"turn", 180},
	// {"out"},
	// {"wait", 200},
	// {"in"},
	// {"turn", 135},
	// {"move", 52},
	// {"move", -52},
	// {"turn", -135},
	// {"move", 5},
	// {"turn", -90},
	// {"move", -53},
	// {"turn", -135},
	// {"wings"},
	// {"move", -5},
	// {"turn", -45},
	// {"move", -12},
};

std::vector<Waypoint> skills = {
	
};

Items items;
Robot robot;
Location maping;

// Runs initialization code. This occurs as soon as the program is started.
void initialize()
{
	pros::lcd::initialize();
	// initialize objects...
	items.initialize();
	robot.initialize(items);
	maping.initialize(robot);
}

// Runs while the robot is in the disabled state
void disabled() { items.stop(); }
// Runs after initialize(), and before autonomous when connected to the Field Management System or the VEX Competition Switch.
void competition_initialize() {
	robot.initialize(items);
}

// Runs the user autonomous code.
void autonomous()
{
	
#ifndef SKILLS
	items.autonmous = true;
	pros::lcd::clear();	
	int count = 0;
	for (Waypoint current_goal : spawn1) {
		pros::lcd::print(1, "%i", count++);

		current_goal.execute_aux_command(&robot);

		CartesianLine robot_line(0, robot.x, robot.y);
		CartesianLine goal_line(0, current_goal.param1 * sin(robot.theta), current_goal.param1 * cos(robot.theta));

		do {
			std::vector<double> vect;
			if (
				current_goal.command == "move" ||
				current_goal.command == "turn" ||
				current_goal.command == "curve" ||
				current_goal.command == "power"
			) vect = maping.updatePID(current_goal, robot_line, goal_line);
			else break;

			robot.set_both_sides(vect[1], vect[0]);
			maping.update();
			pros::delay(AUTON_LOOP_DELAY);
		} while (maping.is_running());

		robot.set_both_sides(0, 0);

		robot.x = 0;
		robot.y = 0;
		robot.theta = 0;
		maping.reset_all();
	}
#else
	robot.set_both_sides(1, 1);
	pros::delay(100);
	items.stop();
	robot.items.cata->move_voltage(120000);
	pros::delay(60000);
	
	pros::lcd::clear();
	int count = 0;
	for (Waypoint current_goal : skills) {

		current_goal.execute_aux_command(&robot);

		CartesianLine robot_line(0, robot.x, robot.y);
		CartesianLine goal_line(0, current_goal.param1 * sin(robot.theta), current_goal.param1 * cos(robot.theta));

		do {
			std::vector<double> vect;
			if (
				current_goal.command == "move" ||
				current_goal.command == "turn" ||
				current_goal.command == "curve"
			) vect = maping.updatePID(current_goal, robot_line, goal_line);
			else break;

			robot.set_both_sides(vect[1], vect[0]);
			pros::lcd::print(0, "%f,%f", goal_line.x, goal_line.y);
			pros::lcd::print(1, "%i, %i", vect[1], vect[0]);
			maping.update();
			pros::delay(AUTON_LOOP_DELAY);
		} while (maping.is_running());

		robot.set_both_sides(0, 0);

		items.master->print(0, 0, "%i", count++);

		robot.x = 0;
		robot.y = 0;
		robot.theta = 0;
		maping.reset_all();
	}
#endif

	// robot.set_hold();
	// robot.set_both_sides(127, 127);
	// pros::delay(1000);
	// items.stop();
	// pros::delay(500);
	// robot.set_both_sides(-127, -127);
	// pros::delay(750);
	// items.stop();
	// pros::delay(1000);
	// robot.set_coast();
}
// Runs the operator control code.`
void opcontrol()
{
	//autonomous(); // disable this during comp...
	items.autonmous = false;
	items.stop();
	items.master->print(0, 0, "LOL");
	// Driver Code:
	while (true)
	{
		// DRIVE TRAIN ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// set speed chassis
		robot.set_speed_chassis(
			items.master->get_analog(ANALOG_LEFT_Y),
			items.master->get_analog(ANALOG_RIGHT_X)
		);
		// actions acording to buttons:
		robot.set_intake(
			items.master->get_digital_new_press(DIGITAL_L1), items.master->get_digital_new_press(DIGITAL_L2),
			items.master->get_digital_new_press(DIGITAL_A)
		);
		robot.set_wings(items.master->get_digital_new_press(DIGITAL_R1));
		robot.set_cata(items.master->get_digital(DIGITAL_Y));
		robot.set_blocker(items.master->get_digital_new_press(DIGITAL_B));

		pros::lcd::print(0, "p:%i t:%i", items.master->get_analog(ANALOG_LEFT_Y), items.master->get_analog(ANALOG_RIGHT_X));
		pros::lcd::print(1, "i-f:%i i-o:%i", items.master->get_digital(DIGITAL_L1), items.master->get_digital(DIGITAL_L2));

		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}

