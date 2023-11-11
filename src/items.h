#ifndef ITEMS_H
#define ITEMS_H

#include "api.h"
#include "variables.h"

// Struct file that contains the objects to all hardware devices.
struct Items {
public:
    int initpos;
    pros::Controller* master;
    pros::Motor* left1;
    pros::Motor* left2;
    pros::Motor* left3;
    pros::Motor* right1;
    pros::Motor* right2;
    pros::Motor* right3;
    pros::Motor* intake_left;
    pros::Motor* intake_right;
    pros::Motor* turret;
    pros::ADIDigitalOut* puncher1;
    pros::ADIDigitalOut* puncher2;
    pros::ADIDigitalOut* pto1;
    pros::ADIDigitalOut* pto2;

    // constructor, connects all harware and ports.
    void initialize();
    // deconstructor, helps end the program without crashing.
    ~Items();
};

#endif // ITEMS_H