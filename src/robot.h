#ifndef ROBOT_H
#define ROBOT_H

#include "depend.h"

class Robot {
private:
    Items items;
    double x;
    double y;
    double theta;

    // the abs raw distances:
    double right_abs_dist();
    double left_abs_dist();
    double center_abs_dist();
public:
    void initialize(Items&);
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
    void set_speed_chassis(int y, int x, long long line, int* speedr = nullptr, int* speedl = nullptr);
    
    void set_intake(int analog);
    void set_turret(int up, int down);
    void set_puncher(int analog);
    double radians_to_degrees(double radians);
    double degrees_to_radians(double degrees);

    // error functions:
    double get_error_r(double);
    double get_error_l(double);
    double get_error_c(double);
};

#endif // ROBOT_H