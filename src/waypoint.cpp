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
    Waypoint goal = {x, y};
    cpp_vect.push_back(goal);
}

int Path::size() {
    return cpp_vect.size();
}

bool Path::goal_reached(Waypoint& goal, double x, double y) {
    double distance = sqrt(x*x + y*y);
    return distance < allowable_error;
}

void Waypoint::execute_command(Robot& robot) {
    if (command == "") {
        robot.items.intake_left->brake();
        robot.items.intake_right->brake();
    } else if (command == "INTAKE") {
        robot.items.intake_piston->set_value(1);
    } else if (command == "OUTAKE") {
        robot.items.intake_piston->set_value(0);
    } else if (command == "WINGS") {
        robot.items.wing_pos = !robot.items.wing_pos;
        robot.items.wings->set_value(robot.items.wing_pos);
    } else if (command == "EXTEND") {
        robot.items.pto_pos = !robot.items.pto_pos;
        robot.items.pto->set_value(robot.items.pto_pos);
    }
}
