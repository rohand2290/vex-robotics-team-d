#include "main.h"
#include "depend.h"

#define UPDATE_COORDS() {\
			std::vector<double> vect = maping.update();\
			robot.x += vect[0];\
			robot.y += vect[1]; }

/*
// const std::vector<Waypoint> spawn1 = {
// 	{27, 32.17},
// 	{42, 0},
// 	{0, 26.2, "WINGS"},
// 	{35, 0},
// 	{0, 27},
// 	{16.97, 16.97},
// 	{24, 0, "INTAKE"},
// 	{52, 0, "MATCH_LOAD"},
// 	{0, 30},
// 	{28.99, 28.99},
// 	{24, 0},
// };

// const std::vector<Waypoint> spawn2 = { // TODO
// 	{27, 32.17},
// 	{42, 0},
// 	{0, 26.2, "WINGS"},
// 	{35, 0},
// 	{0, 27},
// 	{16.97, 16.97},
// 	{24, 0, "INTAKE"},
// 	{52, 0, "MATCH_LOAD"},
// 	{0, 30},
// 	{28.99, 28.99},
// 	{24, 0},
// };
*/

const std::vector<Waypoint> spawn1 = {
	// {4434.333, 3444},
	// {2091.6, -1383.3},
	// {1932, 974},
	{1337, 1213.333},
	{1420.333, -1073.0},
	{1483, 1166.0},
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

// Testers...
inline static void sudo_value_retriever() {

	items.right1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.right2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.right3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    items.left1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.left2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.left3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	pros::lcd::clear();
	items.master->clear();

	bool temp = false;
	bool auton = false;
	// Variables for Smooth Drive:
	int speedr = 0; // speed for right
	int speedl = 0; // speed for left
	// Driver Code:
	auto beg = std::chrono::high_resolution_clock::now();

	double right;
	double left;
	while (true) {
		right = robot.right_abs_dist();
		left = robot.left_abs_dist();

		if (items.master->get_digital(DIGITAL_A)) {
			items.master->print(0, 0, "%f,%f", right, left);
			while (!items.master->get_digital(DIGITAL_B)) {
				items.master->print(1, 0, "Press B to continue");
				pros::delay(AUTON_LOOP_DELAY);
			}
			pros::lcd::clear();
			maping.reset_all();
		}

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
		robot.set_flywheel(
			items.master->get_digital(DIGITAL_DOWN),
			items.master->get_digital(DIGITAL_UP)
		);
		robot.set_wings(items.master->get_digital_new_press(DIGITAL_R1), beg);
		robot.set_pto(items.master->get_digital_new_press(DIGITAL_Y));
		

		// pros::lcd::print(0, "%f", robot.theta);
		// pros::lcd::print(1, "%f", robot.x);
		// pros::lcd::print(2, "%f", robot.y);

		// UPDATE_COORDS();
		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}
inline static void test_motors() {
	pros::lcd::clear();

	items.master->print(0, 0, "Testing flywheel...");
	items.master->print(1, 0, "Press B to continue...");
	while (!items.master->get_digital(DIGITAL_B));
	items.master->clear();

	items.flywheel->move_voltage(12000);
	pros::delay(4000);
	bool pass = false;
	for (int i = 0; i < 1000; ++i) {
		if (items.flywheel->get_actual_velocity() == 600) { pass = true; break; }
	}
	items.stop();
	items.master->clear();
	if (pass) {
		items.master->print(0, 0, "test passed..");
	}

	items.master->print(1, 0, "Press B to continue... (Lift robot up for this one)");
	while (!items.master->get_digital(DIGITAL_B));
	pros::lcd::clear();

	items.right1->move_voltage(12000);
	items.right2->move_voltage(12000);
	items.right3->move_voltage(12000);

	items.left1->move_voltage(12000);
	items.left2->move_voltage(12000);
	items.left3->move_voltage(12000);

	pros::delay(1000);
	pass = false;
	bool pass2 = false;
	for (int i = 0; i < 1000; ++i) {
		if (
			items.right1->get_actual_velocity() == 600 &&
			items.right2->get_actual_velocity() == 600 &&
			items.right2->get_actual_velocity() == 600
		) { pass = true; }
		if (
			items.left1->get_actual_velocity() == 600 &&
			items.left2->get_actual_velocity() == 600 &&
			items.left3->get_actual_velocity() == 600
		) { pass2 = true; }
	}
	items.stop();
	items.master->clear();
	items.master->print(0, 0, pass ? "right test passed..." : "right test failed...");
	items.master->print(1, 0, pass2 ? "left test passed..." : "left test failed...");
	items.master->print(2, 0, "Press B to continue...");

	while (!items.master->get_digital(DIGITAL_B));
	items.master->clear();
	items.master->print(0, 0, "TEST FINISHED");

	TERMINATE();
}

// Runs the user autonomous code.
void autonomous()
{
	pros::lcd::clear();

	// items.right1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // items.right2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // items.right3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // items.left1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // items.left2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // items.left3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	for (Waypoint current_goal : spawn1) {
		// Waypoint current_goal = road.get_latest();
		current_goal.execute_command(robot);
		do {

			std::vector<double> vect = maping.updatePID(current_goal);
			robot.set_both_sides(vect[0], vect[1]);
			pros::delay(AUTON_LOOP_DELAY);

		} while (maping.is_running());


		items.stop();

		maping.reset_all();
	}
}

// Runs the operator control code.
void opcontrol()
{
	//autonomous();
	items.stop();
	// Driver Code:
	auto beg = std::chrono::high_resolution_clock::now();
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
		robot.set_flywheel(
			items.master->get_digital(DIGITAL_DOWN),
			items.master->get_digital(DIGITAL_UP)
		);
		robot.set_wings(items.master->get_digital_new_press(DIGITAL_R1), beg);
		robot.set_pto(items.master->get_digital_new_press(DIGITAL_Y));
		

		// pros::lcd::print(0, "%f", robot.theta);
		// pros::lcd::print(1, "%f", robot.x);
		// pros::lcd::print(2, "%f", robot.y);

		// UPDATE_COORDS();
		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}

