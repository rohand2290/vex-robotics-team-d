#ifndef ROBOT_H
#define ROBOT_H

#include "depend.h"

class Robot {
private:
    Items items;
    double x;
    double y;
    double theta; // in centidegrees.

    // the abs raw distances: (calculate actual distance using rotory sensor)
    double right_abs_dist();
    double left_abs_dist();
    double center_abs_dist();
public:
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
    void set_speed_chassis(int y, int x, long long line, int* speedr = nullptr, int* speedl = nullptr);
    /// @brief Set the intake to in or out
    /// @param analog 1 or 0
    void set_intake(int analog);
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
    /// @brief Gets the error for the PID Algorithm for the right wheels
    /// @param desired desired distance
    /// @return error as double
    double get_error_r(double desired);
    /// @brief Gets the error for the PID Algorithm for the left wheels
    /// @param desired desired distance
    /// @return error as double
    double get_error_l(double desired);
    /// @brief Gets the error for the PID Algorithm for the center wheel (eliminates miscalculations from robots pushing eachother)
    /// @param desired desired distance
    /// @return error as double
    double get_error_c(double desired);
    /// @brief Updates the coordinates after every function call. Call this in a repetitive loop.
    void update_coords();
};

#endif // ROBOT_H