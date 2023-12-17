#ifndef ROBOT_H
#define ROBOT_H

#include "depend.h"

class Robot {
private:
public:
    Items items;
    double x;
    double y;
    double theta; // in degrees.

    int power = 0;
    bool temp1 = 0;
    bool temp2 = 0;

    /// @brief Right absolute distance since start
    /// @return abs distance in inches
    double right_abs_dist();
    /// @brief Left absolute distance since start
    /// @return abs distance in inches
    double left_abs_dist();
    /// @brief Center absolute distance since start
    /// @return abs distance in inches
    double center_abs_dist();
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
    /// @brief Set the flywheel in sticky format.
    /// @param stick input button
    void set_flywheel(int stick, int stick2);
    /// @brief Set the wings in sticky format.
    /// @param stick input button
    void set_wings(int stick, std::chrono::_V2::system_clock::time_point time);
    /// @brief Set the PTO [format not decided]
    /// @param input digital input
    void set_pto(int input);
    /// @brief Converts radians to degrees
    /// @param radians radians as double
    /// @return coresponding degrees as double
    double radians_to_degrees(double radians);
    /// @brief Converts degrees to radians
    /// @param degrees degrees as double
    /// @return radians as double
    double degrees_to_radians(double degrees);
};

#endif // ROBOT_H