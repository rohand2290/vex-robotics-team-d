#ifndef ROBOT_H
#define ROBOT_H

#include "depend.h"

class Robot {
private:
    int intake_state = 0;
    double current_val = 0;
    PID brake_pid;
public:
    Items items;
    double x;
    double y;
    double theta; // in degrees.

    int power = 0;
    bool temp1 = 0;
    bool temp2 = 0;

    /// @brief Gets the cata position in degrees...
    /// @return current position...
    double get_cata_position();
    /// @brief Sets the drive motors to coast...
    void set_coast();
    /// @brief Sets the drive motors to hold...
    void set_hold();
    /// @brief Gets where the robot in pointing in degrees or radians.
    /// @param rad radians or not (degrees)
    /// @return bearing angle
    double get_abs_angle(bool rad = false);
    /// @brief A non default constructor. Should be used in initialize function in main.cpp.
    /// @param i the universal item object's reference
    void initialize(Items& i);
    /// @brief Default destructor...
    ~Robot();
    /// @brief Set the wheel speed of all on the right 
    /// @param analog speed in analog
    void set_right_side(int analog);
    /// @brief  Set the wheel speed of all on the left.
    /// @param analog speed in analog
    void set_left_side(int analog);
    /// @brief Sets both sides of the chasis to a specific speed. Use this if loop delay is small.
    /// @param right analog of the right 
    /// @param left analog of the left
    void set_both_sides(int right, int left);
    /// @brief Set the speed of all the wheels on the chassis, depending on commands given from controller.
    /// @param y the y axis info.
    /// @param x the x axis info.
    /// @param speedr helper var
    /// @param speedl helper var
    void set_speed_chassis(int y, int x);
    /// @brief Set the intake to in or out
    /// @param analog1 1 or 0
    /// @param analog2 1 or 0
    /// @param pist 1 or 0
    void set_intake(int analog1, int analog2, int pist);
    /// @brief Set the wings in sticky format.
    /// @param stick input button
    void set_wings(int stick);
    /// @brief Set the wings on the back in sticky format.
    /// @param stick input button
    void set_wings_back(int stick);
    /// @brief Set the catapult...
    /// @param analog input button
    void set_cata(int analog);
    /// @brief Set the blocker...
    /// @param analog imput button
    void set_blocker(int analog);
    /// @brief Converts radians to degrees
    /// @param radians radians as double
    /// @return coresponding degrees as double
    double radians_to_degrees(double radians);
    /// @brief Converts degrees to radians
    /// @param degrees degrees as double
    /// @return radians as double
    double degrees_to_radians(double degrees);
    /// @brief Brakes the robot untill all inertia is lost...
    void brake();
};

#endif // ROBOT_H