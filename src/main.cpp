#include "main.h"
#include "depend.h"
#include "tests.h"
using namespace std::chrono;

/*
* PLAN:

	Move forward 80 inches
	Intake
	Wings
	135 degrees clockwise
	Move forward 48 inches
	Move backward 5 inches
	135 degrees clockwise
	Move forward 44 inches
	Intake
	Turn 180 degrees
	More forwards 44 inches
	Turn 45 degrees counterclockwise
	Move backwards 54 inches
	Turn 90 degrees counterclockwise
	Move forward 45 inches
	Move back 45 inches
	Turn 45 degrees counterclockwise
	Wings
	Move backwards 8 inches 
	Turn 45 Degrees  counterclockwise 
	Move backwards 24 inches
	Turn 45 clockwise
	Move forward 44 inches
	Turn 45 clockwise
	Move forward 25 inches

*/
std::vector<Waypoint> spawn1 = {
	{"move", 30},
	{"move", -30},
	// {"in"},
	// {"wait", 1000},
	// {"stop"},
	// {"turn", 135},
	// {"move", 48},
	// // {"move", -5},
	// // {"turn", 270},
	// // {"move", 44},
	// // {"in"},
	// // {"wait", 1000},
	// // {"stop"},
	// // {"turn", 90},
	// // {"move", 44},
	// // {"turn", 135},
	// // {""},
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
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous() 
{
	items.autonmous = true;
	pros::lcd::clear();	
	int count = 0;
#ifndef SKILLS
	for (Waypoint current_goal : spawn1) {
#else
	for (Waypoint current_goal : skills) {
#endif
		maping.old_angle = items.imu->get_rotation();
		bool error_type = current_goal.execute_aux_command(&robot);

		CartesianLine robot_line(0, robot.x, robot.y);
		CartesianLine goal_line(0, current_goal.param1 * sin(robot.theta), current_goal.param1 * cos(robot.theta));

		do {
			maping.start_iter = pros::millis();
			std::vector<double> vect;
			if (current_goal.is_motion_command()) {
				vect = maping.updatePID(current_goal, robot_line, goal_line, error_type); 
			}
			else break;

			robot.set_both_sides(vect[1], vect[0]);
			maping.update();
			pros::delay(AUTON_LOOP_DELAY);
		} while (maping.is_running());

		robot.set_both_sides(0, 0);

		maping.cx = 0;
		maping.cy = 0;
		maping.reset_all();
	}
	/*
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
	*/
}
// Runs the operator control code.
void opcontrol()
{
	autonomous(); // disable this during comp...
	items.autonmous = false;
	items.stop();
	// Driver Code:
	while (true)
	{
		// maping.start_iter = pros::millis();
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
		robot.set_wings_back(items.master->get_digital_new_press(DIGITAL_R2));
		robot.set_cata(items.master->get_digital(DIGITAL_Y));
		robot.set_blocker(items.master->get_digital_new_press(DIGITAL_B), items.master->get_digital_new_press(DIGITAL_A));

		// pros::lcd::print(0, "p:%i t:%i", items.master->get_analog(ANALOG_LEFT_Y), items.master->get_analog(ANALOG_RIGHT_X));
		// pros::lcd::print(1, "i-f:%i i-o:%i", items.master->get_digital(DIGITAL_L1), items.master->get_digital(DIGITAL_L2));

		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}

