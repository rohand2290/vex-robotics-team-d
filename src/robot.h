#ifndef ROBOT_H
#define ROBOT_H

#include "depend.h"

class Robot {
private:
    Items items;
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
};

#endif // ROBOT_H