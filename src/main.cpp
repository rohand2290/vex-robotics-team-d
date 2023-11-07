#include "main.h"
#include "depend.h"

int initpos;
pros::Controller* master;
pros::Motor* left1;
pros::Motor* left2;
pros::Motor* left3;
pros::Motor* right1; // 'true' could be removed if neccessary...
pros::Motor* right2;
pros::Motor* right3;
pros::Motor* intake_left;
pros::Motor* intake_right;
pros::Motor* turret;
pros::ADIDigitalOut* puncher1;
pros::ADIDigitalOut* puncher2;
pros::ADIDigitalOut* pto1;
pros::ADIDigitalOut* pto2;

// delete all of the global heap vars (the motors, pistons, etc.):
void deleteHeapVars() {
	delete master;
	delete left1;
	delete left2;
	delete left3;
	delete right1;
	delete right2;
	delete right3;
	delete intake_left;
	delete intake_right;
	delete turret;
	delete puncher1;
	delete puncher2;
	delete pto1;
	delete pto2;
}

// Runs initialization code. This occurs as soon as the program is started.
void initialize() {
	pros::lcd::initialize();
	// initialize objects...
	master = new pros::Controller(pros::E_CONTROLLER_MASTER);
	left1  = new pros::Motor (LEFT_WHEELS_PORT_1, true);
	left2  = new pros::Motor (LEFT_WHEELS_PORT_2, true);
	left3  = new pros::Motor (LEFT_WHEELS_PORT_3);
	right1 = new pros::Motor (RIGHT_WHEELS_PORT_1);
	right2 = new pros::Motor (RIGHT_WHEELS_PORT_2);
	right3 = new pros::Motor (RIGHT_WHEELS_PORT_3, true);

	intake_left = new pros::Motor (INTAKE_PORT_LEFT);
	intake_right = new pros::Motor (INTAKE_PORT_RIGHT, true);

	turret = new pros::Motor(TURRET_PORT);

	puncher1 = new pros::ADIDigitalOut (PUNCHER_PORT_1, 0);
	puncher2 = new pros::ADIDigitalOut (PUNCHER_PORT_2, 0);

	initpos = 1;
	pto1 = new pros::ADIDigitalOut (PTO_PORT_1, initpos);
	pto2 = new pros::ADIDigitalOut (PTO_PORT_2, initpos);
}

// Runs while the robot is in the disabled state
void disabled() {}

// Runs after initialize(), and before autonomous when connected to the Field Management System or the VEX Competition Switch.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous() {}

// Runs the operator control code.
void opcontrol() {
	pros::lcd::print(1, "starting code...");
	pros::delay(1000);

	bool temp = false;
	// Variables for Smooth Drive:
	int speedr = 0; // speed for right
	int speedl = 0; // speed for left
	// Driver Code:
	while (true) {
		int power = master->get_analog(ANALOG_LEFT_Y);
		int turn = master->get_analog(ANALOG_RIGHT_X);
		int left = (power + (turn * TURN_PERCENT)) * MOTOR_PERCENT;
		int right = (power - (turn * TURN_PERCENT)) * MOTOR_PERCENT;
		
		// set the wheels...
		// left side:
		if (speedl >= left) speedl -= SMOOTH_CONSTANT;
		else if (speedl < left) speedl += SMOOTH_CONSTANT;
		// right side:
		if (speedr >= right) speedr -= SMOOTH_CONSTANT;
		else if (speedr < right) speedr += SMOOTH_CONSTANT;

		left1->move(speedl); left2->move(speedl); left3->move(speedl);
		right1->move(speedr); right2->move(speedr); right3->move(speedr);

		// actions acording to buttons:
		if (master->get_digital(DIGITAL_L1)) {
			intake_left->move(INTAKE_IN_SPEED);
			intake_right->move(INTAKE_IN_SPEED);
		} else if (master->get_digital(DIGITAL_R1)) {
			intake_left->move(INTAKE_OUT_SPEED);
			intake_right->move(INTAKE_OUT_SPEED);
		}
		if (master->get_digital(DIGITAL_UP)) {
			turret->move(TURRET_SPEED);
		} else {
			turret->move(0);
		}
		if (master->get_digital(DIGITAL_DOWN)) {
			turret->move(-TURRET_SPEED);
		} else {
			turret->move(0);
		}

		puncher1->set_value(master->get_digital(DIGITAL_A));
		puncher2->set_value(master->get_digital(DIGITAL_A));

		// print sensory info to the sceen:
		pros::lcd::print(1, "left: %i", left);
		pros::lcd::print(2, "right: %i", right);
		pros::lcd::print(3, "l1 (intake): %i | r1 (shoot): %i", 
			master->get_digital(DIGITAL_L1),
			master->get_digital(DIGITAL_R1));
		pros::lcd::print(4, "up (turret): %i | dn (turret): %i", 
			master->get_digital(DIGITAL_UP),
			master->get_digital(DIGITAL_DOWN));
		pros::lcd::print(5, "A: %i | initpos (pto): %i", 
			master->get_digital(DIGITAL_A),
			initpos);

		pros::delay(OPCONTROL_LOOP_DELAY);

		if (master->get_digital(DIGITAL_Y)) {
			if (!temp) temp = true;
			else continue;			
			initpos = !initpos;
			pto1->set_value(initpos);
			pto2->set_value(initpos);
		} if (!master->get_digital(DIGITAL_Y)) temp = false;
	}
	deleteHeapVars();
}
