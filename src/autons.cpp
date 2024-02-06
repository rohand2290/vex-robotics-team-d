#include "depend.h"

PID::PID(double kp, double ki, double kd) {
    KP = kp;
    KI = ki;
    KD = kd;
    integral = 0;
    prev_error = 0;
}

PID::PID() {}

void PID::initialize(double kp, double ki, double kd) {
    KP = kp;
    KI = ki;
    KD = kd;
    integral = 0;
    prev_error = 0;
}

double PID::update(double error) {
    double P = error * KP;
	integral += error;
	if (abs(error) < ACCURACY) {
		integral = 0;
	}
    if (error > MAX_I_VAL || error < MIN_I_VAL) {
        integral = 0;
    }
	double I = integral * KI;
	double D = error - prev_error;
	D *= KD;

	return P + I + D;
}

void PID::reset() {
    integral = 0;
    prev_error = 0;
}

Autotuner::Autotuner(Location& l, Robot& r): maping(l), robot(r) {}

void Autotuner::run(Waypoint command) {
    std::vector<Waypoint> s = {
		{"move", 10} // EDIT THIS TO TUNE SEPERATE PIDs: (assume odom is tuned)
	};
    std::vector<double> proportional;
    std::vector<double> integral;
    std::vector<double> derivative;

	// step 1: get raw data.
	maping.reset_all();
	robot.items.autonmous = true;
	int count = 0;
	bool error_type = false;
	maping.old_angle = robot.items.imu->get_rotation();
	error_type = command.execute_aux_command(&robot);
	CartesianLine robot_line(0, robot.x, robot.y);
	CartesianLine goal_line(0, command.param1 * sin(robot.theta), command.param1 * cos(robot.theta));
	do
	{
		maping.start_iter = pros::millis();
		std::vector<double> vect;
		if (command.is_motion_command()) vect = maping.updatePID(command, robot_line, goal_line, error_type);
		else break;
		robot.set_both_sides(vect[1], vect[0]);
		maping.update();
		pros::delay(5);

        

	} while (maping.is_running());
	robot.set_both_sides(0, 0);
	maping.cx = 0;
	maping.cy = 0;
	maping.reset_all();
}
