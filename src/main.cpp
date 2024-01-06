#include "main.h"
#include "depend.h"
#include "tests.h"

std::vector<Waypoint> spawn1 = {
	{"move", 48},
	{"move", -48},
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
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous()
{
	items.autonmous = true;
	pros::lcd::clear();

	items.master->print(0, 0, "GO!");
	for (Waypoint current_goal : spawn1) {

		current_goal.execute_aux_command(robot);
		CartesianLine robot_line(0, 0, 0);
		CartesianLine goal_line(0, 0, 0);

		do {
			std::vector<double> vect;
			if (
				current_goal.command == "move" ||
				current_goal.command == "turn" ||
				current_goal.command == "curve"
			) vect = maping.updatePID(current_goal, robot_line, goal_line);
			else break;

			robot.set_both_sides(vect[1], vect[0]);
			maping.update();
			pros::delay(AUTON_LOOP_DELAY);
		} while (maping.is_running());

		items.stop();

		robot.x = 0;
		robot.y = 0;
		robot.theta = 0;
		maping.reset_all();
	}
}

// Runs the operator control code.
void opcontrol()
{
	//autonomous(); // disable this during comp...
	items.autonmous = false;
	items.stop();
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

