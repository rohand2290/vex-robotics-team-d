#include "depend.h"

void Path::initialize(double x, double y) {
    add_queue(x, y);
}

void Path::initialize(const std::vector<Waypoint> a) {
    cpp_vect = a;
}

Waypoint Path::get_latest() {
    return cpp_vect[0];
}

void Path::pop_latest() {
    if (cpp_vect.size() > 0) {
        cpp_vect.erase(cpp_vect.begin());
    } 
}

void Path::add_queue(Waypoint goal) {
    cpp_vect.push_back(goal);
}

void Path::add_queue(double x, double y) {
    Waypoint goal = {"", y};
    cpp_vect.push_back(goal);
}

int Path::size() {
    return cpp_vect.size();
}

bool Path::goal_reached(Waypoint& goal, double x, double y) {
    double distance = sqrt(x*x + y*y);
    return distance < allowable_error;
}

bool Waypoint::execute_aux_command(Robot* robot) {
    if (command == "move" || command == "turn" || command == "curve" || command == "swing" || command == "raw");
    else if (command == "stop") {
        robot->items.intake_left->brake();
        robot->items.intake_right->brake();
    } else if (command == "in") {
        robot->items.intake_left->move(255);
        robot->items.intake_right->move(255);
    } else if (command == "out") {
        robot->items.intake_left->move(-255);
        robot->items.intake_right->move(-255);
    } else if (command == "wings") {
        robot->items.wing_pos = !robot->items.wing_pos;
        robot->items.wings->set_value(robot->items.wing_pos);
    } else if (command == "bwings") {
        robot->items.wing_back_pos = !robot->items.wing_back_pos;
        robot->items.wings_back->set_value(robot->items.wing_back_pos);
    } else if (command == "rise") {
        robot->items.pto->set_value(1);
    } else if (command == "fall") {
        robot->items.pto->set_value(0);
    } else if (command == "coast") {
        robot->set_coast();
    } else if (command == "hold") {
        robot->set_hold();
    } else if (command == "wait") {
        pros::delay(param1);
    } else if (command == "spamcata") {
        while (true) {
            robot->items.cata->move_voltage(120000);
            pros::delay(AUTON_LOOP_DELAY);
        }
    } else if (command == "cata") {
        robot->run_cata_x_times(param1);
    } else if (command == "tcata") {
        for (int i = 0; i < param1; ++i) {
            robot->items.cata->move_voltage(120000);
            pros::delay(1);
        }
        robot->items.cata->brake();
    } 
    else if (command == "precise") {
        return false;
    } else if (command == "pass") {
        return true;
    }
    return false;
}

bool Waypoint::is_motion_command() {
    return 
        command == "move" ||
		command == "turn" ||
		command == "curve" ||
		command == "power" ||
        command == "swing" || 
        command == "raw";
}