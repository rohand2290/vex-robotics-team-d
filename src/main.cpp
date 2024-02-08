#include "main.h"
#include "depend.h"
#include "tests.h"
using namespace std::chrono;

/*
* PLAN:

	Move forward 180 inches
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
	Turn 45 Degrees counterclockwise
	Move backwards 24 inches
	Turn 45 clockwise
	Move forward 44 inches
	Turn 45 clockwise
	Move forward 25 inches

*/
static double d2r(double x) {
	return 0.0174533 * x;
}
static void reverse_turns(std::vector<Waypoint>& v) {
	for (Waypoint& i : v) {
		if (i.command == "turn") i.param1 *= -1;
	}
}

std::vector<Waypoint> spawn1 = {
	/////// ================= NORMAL AUTON
	{"pass"},
	{"in"},
	{"move", 62},
	{"turn", 120},
	{"stop"},
	{"wings"},
	{"power", 120000, 750},
	{"wings"},
	{"stop"},
	{"move", -10},
	{"turn", 130},
	{"move", 25},
	{"in"},
	{"turn", -160},
	{"power", 120000, 1100},
	{"stop"},
	//works till here...
	{"move", -5},
	{"turn", 90},
	{"move", 51.25},
	{"turn", 90},
	{"in"},
	{"move", 29},
	{"wait", 500},
	{"stop"},
	{"move", -34},
	{"turn", -45},
	{"bwings"},
	{"move", -20},
	{"turn", -60},
	{"bwings"},
	{"power", -120000, 1000},
	{"move", 5},
	{"turn", 180},
	{"power", 120000, 1000},
	{"move", -5},
	// ==================================================== SKILLS:
	// {"pass"},
	// {"bwings"},
	// {"move", -12},
	// {"turn", 60},
	// {"bwings"},
	// {"power", -63, 1000},
	// {"move", 12},
	// {"turn", 75},
	// {"power", 10, 200},
	// {"hold"},
	// {"tcata", 30000}, // change this to 30000 during actual match.
	// {"coast"},
	// //// done matchloading 
	// {"move", -40},
	// {"turn", 95},
	// {"wings"},
	// {"precise"},
	// {"out"},
	// {"move", 40},
	// {"pass"},
	// {"turn", 90},
	// {"wings"},
	// {"power", 120000, 1600},
	// {"stop"},
	// {"wings"},
	// //// we are over the barrier...
	// {"wait", 1000},
	// {"precise"},
	// {"turn", 180},
	// {"pass"},
	// {"bwings"},
	// {"power", -63, 1000},
	// {"bwings"},
	// {"move", 30},
	// {"turn", -90},
	// {"move", 25},
	// {"turn", 70},
	// {"bwings"},
	// {"power", -63, 1500},
	// {"move", 10},
	// // going to go climb
	// {"turn", 90},
	// {"move", 60},
	// {"turn", -90},
	// {"rise"},
	// {"move", 42},
	// {"fall"},
	// ================================================ NORMAL WINPOINT:
	// 	{"pass"},
	// 	{"bwings"},
	// 	{"move", -10},
	// 	{"turn", 45},
	// 	{"bwings"},
	// 	{"wait", 500},
	// 	{"power", -120000, 500},
	// 	{"move", 5},
	// 	{"turn", -10},
	// 	{"move", 5},
	// 	{"turn", -195},
	// 	{"move", -27.5},
	// 	{"turn", -45},
	// 	{"move", -33},

	// ================================================ DISRUPTION WINPOINT:
	// {"precise"},
	// {"move", 52},
	// {"turn", 80},
	// {"wings"},
	// {"out"},
	// {"pass"},
	// {"power", 127, 500},
	// {"move", -21},
	// {"wings"},
	// {"turn", 105},
	// {"in"},
	// {"move", 43},
	// {"stop"},
	// {"turn", 90},
	// {"move", 9.5},
	// {"turn", 60},
	// {"power", 63, 750},
	// {"move", -15},
	// {"turn", -45},
	// {"bwings"},
	// {"move", -20},
	// {"turn", -60},
	// {"bwings"},
	// {"turn", -165},
	// {"precise"},
	// {"move", 36},
};
// ============================================== Random thing that doesnt cause syntax errors ========================
std::vector<Waypoint> skills = {};

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
	maping.reset_all();
	items.autonmous = true;
	int count = 0;
	bool error_type = false;
#ifndef SKILLS
	for (Waypoint current_goal : spawn1)
	{
#else
	for (Waypoint current_goal : skills)
	{
#endif
		maping.old_angle = items.imu->get_rotation();
		error_type = current_goal.execute_aux_command(&robot);
		CartesianLine robot_line(0, robot.x, robot.y);
		CartesianLine goal_line(0, current_goal.param1 * sin(robot.theta), current_goal.param1 * cos(robot.theta));
		do
		{
			maping.start_iter = pros::millis();
			std::vector<double> vect;
			if (current_goal.is_motion_command())
			{
				vect = maping.updatePID(current_goal, robot_line, goal_line, error_type);
			}
			else
				break;

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
static void get_raw_coordinates() {
        items.master->clear();
        int state;
        Location maping;
        maping.initialize(robot);
        while (true) {  
			state = 0;         
            double x;
            double y;
			items.master->print(0, 0, "reading...");
            while (!state) {
                maping.update();
                if (items.master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) state = !state;

                x = maping.cx;
                y = maping.cy;
                pros::delay(5);           
            }
            maping.reset_all();
            items.master->print(0, 0, "%f,%f", x, y);
            while (!state) {
                if (items.master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) state = !state;
                pros::delay(20);
            }
			items.master->clear();
        }
    }
static void get_stats(Location& maping) {
        while (true) {
            maping.update();
            pros::lcd::print(0, "%f", items.imu->get_rotation());
            pros::lcd::print(1, "%f", maping.cx);
            pros::lcd::print(2, "%f", maping.cy);

            pros::delay(20);
        }
    }
static void test_pid() {
	std::vector<Waypoint> spawn1 = {
		// TEST MOVEMENT:
		{"move", 30},
		//// or TEST TURN
		// {"turn", 90},
	};
	maping.reset_all();
	items.autonmous = true;
	int count = 0;
	bool error_type = false;
	for (Waypoint current_goal : spawn1)
	{
		maping.old_angle = items.imu->get_rotation();
		error_type = current_goal.execute_aux_command(&robot);
		CartesianLine robot_line(0, robot.x, robot.y);
		CartesianLine goal_line(0, current_goal.param1 * sin(robot.theta), current_goal.param1 * cos(robot.theta));
		do
		{
			maping.start_iter = pros::millis();
			std::vector<double> vect;
			if (current_goal.is_motion_command())
			{
				vect = maping.updatePID(current_goal, robot_line, goal_line, error_type);
			}
			else
				break;

			robot.set_both_sides(vect[1], vect[0]);
			maping.update();
			pros::delay(AUTON_LOOP_DELAY);
		} while (true); // this is why this is a test

		robot.set_both_sides(0, 0);

		maping.cx = 0;
		maping.cy = 0;
		maping.reset_all();
	}
}

void opcontrol()
{
	//autonomous(); // disable if testing autonomous
	//get_raw_coordinates();
	//get_stats(maping);

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
			items.master->get_analog(ANALOG_RIGHT_X));
		// actions acording to buttons:
		robot.set_intake(
			items.master->get_digital(DIGITAL_L1), items.master->get_digital(DIGITAL_L2),
			items.master->get_digital_new_press(DIGITAL_A));
		robot.set_wings(items.master->get_digital_new_press(DIGITAL_R1));
		robot.set_wings_back(items.master->get_digital_new_press(DIGITAL_R2));
		robot.set_cata(items.master->get_digital(DIGITAL_Y));
		robot.set_blocker(items.master->get_digital_new_press(DIGITAL_B), items.master->get_digital_new_press(DIGITAL_A));
		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}
