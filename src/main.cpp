#include "main.h"
#include "depend.h"

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

	// pros::lcd::register_btn1_cb(on_center_button);
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

	pros::Motor intake_left (INTAKE_PORT_LEFT);
	pros::Motor intake_right (INTAKE_PORT_RIGHT, true);

	pros::Motor turret (TURRET_PORT);

	pros::ADIDigitalOut puncher1 (PUNCHER_PORT_1, 0);
	pros::ADIDigitalOut puncher2 (PUNCHER_PORT_2, 0);

	pros::lcd::print(1, "starting code...");
	pros::delay(1000);

	// Driver Code:
	while (true) {

		int power = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		int left = (power + turn) * MOTOR_PERCENT;
		int right = (power - turn) * MOTOR_PERCENT;
		
		// set the wheels...
		left1.move(left); left2.move(left); left3.move(left);
		right1.move(right); right2.move(right); right3.move(right);

		if (master.get_digital(DIGITAL_L1)) {
			intake_left.move(INTAKE_IN_SPEED);
			intake_right.move(INTAKE_IN_SPEED);
		} else if (master.get_digital(DIGITAL_R1)) {
			intake_left.move(INTAKE_OUT_SPEED);
			intake_right.move(INTAKE_OUT_SPEED);
		}
		if (master.get_digital(DIGITAL_UP)) {
			turret.move(TURRET_SPEED);
		} else if (master.get_digital(DIGITAL_DOWN)) {
			turret.move(-TURRET_SPEED);
		}
		puncher1.set_value(master.get_digital(DIGITAL_A));
		puncher2.set_value(master.get_digital(DIGITAL_A));

		pros::lcd::print(1, "left: %i", left);
		pros::lcd::print(2, "right: %i", right);
		pros::lcd::print(3, "l1 (intake): %i | r1 (shoot): %i", 
			master.get_digital(DIGITAL_L1),
			master.get_digital(DIGITAL_R1));
		pros::lcd::print(4, "up (turret): %i | dn (turret): %i", 
			master.get_digital(DIGITAL_UP),
			master.get_digital(DIGITAL_DOWN));
		pros::lcd::print(5, "A: %i", master.get_digital(DIGITAL_A));

		pros::delay(OPCONTROL_LOOP_DELAY);
	}
}
