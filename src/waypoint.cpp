#include "depend.h"

void Path::initialize(double x, double y) {
    add_queue(x, y);
}

Waypoint Path::get_latest() {
    return cpp_vect[0];
}

void Path::pop_latest() {
    if (cpp_vect.size() >= 0) {
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
    return distance <= allowable_error;
}

void Waypoint::execute_command(Robot& robot) {
    // =================================== TODO ====================================
    if (command == "") {
        robot.items.intake_left->brake();
        robot.items.intake_right->brake();
    }
}