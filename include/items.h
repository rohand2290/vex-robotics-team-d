#ifndef ITEMS_H
#define ITEMS_H

#include "api.h"
#include "variables.h"

// Struct file that contains the objects to all hardware devices.
struct Items {
public:
    int wing_pos;
    int intake_pos;
    int wing_back_pos;
    int autonmous;
    pros::Controller* master;
    pros::Motor* left1; // front
    pros::Motor* left2; // middle
    pros::Motor* left3; // back
    pros::Motor* right1; // front
    pros::Motor* right2; // middle
    pros::Motor* right3; // back
    pros::Motor* intake_left;
    pros::Motor* intake_right;
    pros::Motor* cata;
    pros::ADIDigitalOut* wings;
    pros::ADIDigitalOut* wings_back;
    pros::ADIDigitalOut* pto;
    pros::Rotation* encoder_left;
    pros::Rotation* encoder_right;
    pros::Rotation* encoder_center;
    pros::Rotation* encoder_cata;
    pros::IMU* imu;

    /// @brief Non-default constructor, connects all harware and ports.
    void initialize();
    /// @brief Stops all motors systematically.
    void stop();
    /// @brief Deconstructor, helps end the program without crashing.
    ~Items();
};

#endif // ITEMS_H