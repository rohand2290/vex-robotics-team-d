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

void Waypoint::execute_aux_command(Robot& robot) {
    if (command == "stop") {
        robot.items.intake_left->brake();
        robot.items.intake_right->brake();
    } else if (command == "in") {
        robot.items.intake_left->move(255);
        robot.items.intake_right->move(255);
    } else if (command == "out") {
        robot.items.intake_left->move(-255);
        robot.items.intake_right->move(-255);
    } else if (command == "wings") {
        robot.items.wing_pos = !robot.items.wing_pos;
        robot.items.wings->set_value(robot.items.wing_pos);
    } else if (command == "rise") {
        robot.items.lift1->set_value(1);
        robot.items.lift2->set_value(1);
    } else if (command == "fall") {
        robot.items.lift1->set_value(0);
        robot.items.lift2->set_value(0);
    } else if (command == "coast") {
        robot.items.right1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot.items.right2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot.items.right3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot.items.left1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot.items.left2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        robot.items.left3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    } else if (command == "hold") {
        robot.items.right1->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        robot.items.right2->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        robot.items.right3->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        robot.items.left1->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        robot.items.left2->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        robot.items.left3->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    } else if (command == "wait") {
        pros::delay(param1);
    }
}
