#include "main.h"
#include "api.h"
#include "variables.h"

// A callback function for LLEMU's center button.
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

// Runs initialization code. This occurs as soon as the program is started.
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Version 1.0");

	pros::lcd::register_btn1_cb(on_center_button);

	// initialize the chassis motors:

}

// Runs while the robot is in the disabled state
void disabled() {}

// Runs after initialize(), and before autonomous when connected to the Field Management System or the VEX Competition Switch.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous() {}

// Runs the operator control code.
void opcontrol() {
	// Objects:
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left1   (LEFT_WHEELS_PORT_1);
	pros::Motor left2   (LEFT_WHEELS_PORT_2);
	pros::Motor left3   (LEFT_WHEELS_PORT_3);
	pros::Motor right1  (RIGHT_WHEELS_PORT_1, true); // 'true' could be removed if neccessary...
	pros::Motor right2  (RIGHT_WHEELS_PORT_2, true);
	pros::Motor right3  (RIGHT_WHEELS_PORT_3, true);

	// Driver Code:
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

#ifdef TANK // Compile tank style controls:
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left1 = left; left2 = left; left3 = left;
		right1 = right; right2 = right; right3 = right;
#else       // Compile arcade style controls:
		int power = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		int left = power + turn;
		int right = power - turn;
		
		left1.move(left); left2.move(left); left3.move(left);
		right1.move(right); right2.move(right); right3.move(right);
#endif

		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}