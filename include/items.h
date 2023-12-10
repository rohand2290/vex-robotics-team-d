#ifndef ITEMS_H
#define ITEMS_H

#include "api.h"
#include "variables.h"

// Struct file that contains the objects to all hardware devices.
struct Items {
public:
    int initpos;
    int initpos2;
    pros::Controller* master;
    pros::Motor* left1; // front
    pros::Motor* left2; // middle
    pros::Motor* left3; // back
    pros::Motor* right1; // front
    pros::Motor* right2; // middle
    pros::Motor* right3; // back
    pros::Motor* intake_left;
    pros::Motor* intake_right;
    pros::Motor* flywheel;
    pros::ADIDigitalOut* pto;
    pros::ADIDigitalOut* wings;
    pros::Rotation* encoder_left;
    pros::Rotation* encoder_right;
    pros::Rotation* encoder_center;

    /// @brief Non-default constructor, connects all harware and ports.
    void initialize();
    void stop();
    /// @brief Deconstructor, helps end the program without crashing.
    ~Items();
};

#endif // ITEMS_H