#include "main.h"
#include "depend.h"
#include "tests.h"
#include <math.h>

using namespace std::chrono;

static double d2r(double x) {
	return 0.0174533 * x;
}
static void reverse_turns(std::vector<Waypoint>& v) {
	for (Waypoint& i : v) {
		if (i.command == "turn") i.param1 *= -1;
	}
}

std::vector<Waypoint> spawn1 = {
	/////// ======================== TEST:
	/* {"turn", 90},
	 {"turn", 180},
	 {"turn", -180},
	 {"turn", 315},*/
	 /////// ================= NORMAL AUTON
	 //{"precise"},
	 //{"in"},
	 //{"move", 59},
	 //{"turn", 130},
	 //{"pass"},
	 //{"stop"},
	 //{"wings"},
	 //{"power", 120000, 750},
	 //{"wings"},
	 //{"stop"},
	 //{"move", -10},
	 //{"turn", 130},
	 //{"in"},
	 //{"move", 25},
	 //{"wait", 500},
	 //{"turn", -140},
	 //{"power", 120000, 1300},
	 //{"stop"},
	 ////works till here...
	 //{"move", -9},
	 //{"turn", 92.5},
	 //{"move", 45},
	 //{"turn", 91.25},
	 //{"in"},
	 //{"move", 25},
	 //{"wait", 650},
	 //{"stop"},
	 //{"move", -17.5},
	 //{"turn", -35},
	 //{"move", -17.5},
	 //{"turn", -45},
	 //{"bwings"},
	 //{"move", -15},
	 //{"turn", -55},
	 //{"bwings"},
	 //{"power", -120000, 1000},
	 //{"move", 10},
	 //{"turn", 180},
	 //{"power", 120000, 1000},
	 //{"move", -15},
	 ///// ==================================================== SKILLS:
	 //{"power", 127, 3000},
	 //{"move", -9},
	 //{"turn", 77},
	 //{"move", 4.5},
	 //{"bwings"},
	 //{"move", -2.45},
	 //{"wait", 3000},
	 ////{"tcata", 20000},
	 //{"bwings"},
	 //{"move", 36},
	 //{"turn", -85.5},
	 //{"wings"},
	 //{"move", 80},
	 //{"move", -10},
	 //{"turn", -90},
	 //{"move", 18},
	 //{"turn", 92.25},
	 //{"move", 20},
	 //{"turn", 95},
	 //{"move", 36},
	 //{"turn", 30},
	 //{"move", 36},
	 //{"turn", 230},
	 //{"power", -127, 1500},
	 //{"move", 4},
	 //{"wait", 200},
	 //{"turn", -92.5},
	 //{"move", 35},
	 //{"turn", 91},
	 //{"move", -24},
	 //{"turn", -91},
	 //{"bwings"},
	 //{"power", -127, 1000},
	 //{"bwings"},
	 //{"move", 25},
	 //{"turn", 91},
	 //{"move", -30},
	 //{"turn", -120},
	 //{"bwings"},
	 //{"power", -127, 1000},
	 //{"bwings"},
	 //{"move", 25},
	 //{"bwings"},
	 //{"power", -127, 2000},
	 //{"move", -10},
	 //{"bwings"},
	 //{"stop"},
	 //// ================================================ NORMAL WINPOINT:
	 // {"in"},
	 // {"pass"},
	 // {"bwings"},
	 // {"move", -10},
	 // {"stop"},
	 // {"turn", 60},
	 // {"bwings"},
	 // {"wait", 500},
	 // {"power", -120000, 500},
	 // {"move", 5},
	 // {"turn", -10},
	 // {"move", 5},
	 // {"precise"},
	 // {"turn", -195},
	 // {"move", -27.5},
	 // {"turn", -45},
	 // {"out"},
	 // {"move", -33},
	 // {"stop"},
	 ////// ================================================ DISRUPTION WINPOINT:
	 //{"precise"},
	 //{"out"},
	 //{"move", 43},
	 //{"wings"},
	 //{"turn", 80},
	 //{"pass"},
	 //{"power", 127, 550},
	 //{"move", -21},
	 //{"wings"},
	 //{"turn", 110},
	 //{"in"},
	 //{"move", 36.5},
	 //{"stop"},
	 //{"turn", 105},
	 //{"move", 12},
	 //{"turn", 60},
	 //{"move", 15},
	 //{"turn", 55},
	 //{"out"},
	 //{"power", 127, 1000},
	 //{"stop"},
	 //{"move", -12},
	 //{"turn", 135},
	 //{"bwings"},
	 //{"move", 19.5},
	 //{"precise"},
	 //{"turn", -50},
	 //{"bwings"},
	 //{"pass"},
	 //{"move", -5},
	 //{"turn", -20},
	 //{"precise"},
	 ////{"turn", -165},
	 //{"move", 27},
	 //{"hold"},
	 //{"wait", 2000},
	 //{"coast"},
	 ////// ================================================== NON-DISRUPTION AUTON:
	 {"pass"},
	 {"bwings"},
	 {"move", -15},
	 {"turn", -95},
	 {"turn", 50},
	 {"bwings"},
	 {"power", -127, 750},
	 {"move", 7.5},
	 {"turn", 100},
	 {"move", 48.5},
	 {"turn", 135},
	 {"wings"},
	 {"power", 127, 1550},
	 {"move", -10},
	 {"wings"},
	 {"turn", -160},
	 {"move", 20},
	 {"turn", 180},
	 {"power", 127, 1550},
	 {"move", -10},
};// ============================================== Random thing that doesnt cause syntax errors ========================
std::vector<Waypoint> SKILLS = {};

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

			robot.set_both_sides(vect[1], vect[0], (current_goal.command == "turn") ? false : true);
			maping.update();
			pros::delay(AUTON_LOOP_DELAY);
		} while (maping.is_running());

		robot.set_both_sides(0, 0);

		// I dont think we need to reset this...
		maping.cx = 0;
		maping.cy = 0;
		maping.reset_all((current_goal.command == "turn" && current_goal.param2 == 1) ? true : false);
	}

	// robot.set_both_sides(127, 127);
	// pros::delay(1500);
	// items.stop();
	// pros::delay(500);
	// robot.set_both_sides(-127, -127);
	// pros::delay(750);
	// items.stop();
	// pros::delay(1000);
	// robot.set_coast();
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
		{"turn", 180},
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

			robot.set_both_sides(vect[1], vect[0], (current_goal.command == "turn") ? false : true);
			maping.update();
			pros::delay(AUTON_LOOP_DELAY);
		} while (true); // this is why this is a test
	}
}
static void get_mech_advantage() {
	pros::lcd::clear();
	pros::lcd::print(0, "Please move the bot 30 inches.");
	pros::lcd::print(1, "Press A to continue.");
	items.master->clear();
}
void opcontrol()
{
	//test_pid();
	//autonomous(); // disable if testing autonomous
	//get_raw_coordinates();
	//get_stats(maping);
	/*Tester test(robot, items);
	test.test_chassis();*/
	/*Autotuner a(maping, robot);
	a.run({"move", 20}, 450);*/

	items.autonmous = false;
	items.stop();
	items.master->clear();
	pros::lcd::print(0, "I am in opcontrol");
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
		robot.set_cata(items.master->get_digital(DIGITAL_Y)
			,items.master->get_digital_new_press(DIGITAL_X));
		robot.set_blocker(items.master->get_digital_new_press(DIGITAL_B), items.master->get_digital_new_press(DIGITAL_A));
		/*items.intake_left->move(127);
		items.intake_right->move(127);*/
		pros::lcd::print(0, "%f", items.imu->get_rotation());

		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}
