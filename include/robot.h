#ifndef ROBOT_H
#define ROBOT_H

#include "depend.h"

class Robot {
private:
public:
    Items items;
    // the abs raw distances: (calculate actual distance using rotory sensor)
    double right_abs_dist();
    double left_abs_dist();
    double center_abs_dist();
    /// @brief Gets where the robot in pointing in degrees or radians.
    /// @param rad radians or not (degrees)
    /// @return bearing angle
    double get_abs_angle(bool rad = false);

    double x;
    double y;
    int theta; // in centidegrees.

    /// @brief A non default constructor. Should be used in initialize function in main.cpp.
    /// @param i the universal item object's reference
    void initialize(Items& i);
    ~Robot();

    /// @brief  Set the wheel speed of all on the right 
    /// @param analog speed in analog
    void set_right_side(int analog);
    /// @brief  Set the wheel speed of all on the left.
    /// @param analog speed in analog
    void set_left_side(int analog);
    /// @brief Set the speed of all the wheels on the chassis, depending on commands given from controller.
    /// @param y the y axis info.
    /// @param x the x axis info.
    /// @param speedr helper var
    /// @param speedl helper var
    void set_speed_chassis(int y, int x, long long line, int& speedr, int& speedl);
    /// @brief Set the intake to in or out
    /// @param analog1 1 or 0
    /// @param analog2 1 or 0
    void set_intake(int analog1, int analog2);
    /// @brief Set the turret move based on up or down.
    /// @param up make turret go up (1, 0)
    /// @param down make turret go down (1, 0)
    void set_turret(int up, int down);
    /// @brief Set the puncher, either shot (on) or reload (off)
    /// @param analog puncher value (1, 0)
    void set_puncher(int analog);
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