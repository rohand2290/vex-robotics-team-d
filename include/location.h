#ifndef LOCATION_H
#define LOCATION_H

#include "depend.h"
#include "filters.h"

class IMULocation {
private:
    Robot& robot;
    Items& items;
    double x;
    double y;

    // helper vectors:
    VectorXD<2> prev_vel;
    VectorXD<2> prev_dis;

    // filter:
    Kalman1VFilter* filtx;
    Kalman1VFilter* filty;
public:
    /// @brief Default constructor... Uses purely IMU based navigation.
    /// @param r Robot instance.
    IMULocation(Robot& r);
    /// @brief Computes the current coordinates...
    void compute();
    /// @brief Returns current Dispacement in X-axis
    /// @return Displacement in meters...
    double getX();
    /// @brief Returns current Dispacement in Y-axis
    /// @return Displacement in meters...
    double getY();
    /// @brief default deconstructor...
    ~IMULocation();
};

class Location {
private:
    // robot coordinates
    double *x;
    double *y;
    // robot class:
    Robot* robot;
    // old cached variables:
	double old_l = 0;
	double old_r = 0;
    double old_th = 0;
	// change variables. Show change from previous set of data.
	double rel_l = 0;
	double rel_r = 0;
    double rel_th = 0;
    // PID:
    double error = 0;
    double error_turn_casual = 0;
    double error_swing = 0;
    // PID timer:
    int timer = 0;
    int abs_timer = 0;
    // PID Class:
    PID dis;
    PID turn_casual;
    PID swing;
    bool is_bashing = false;
public:
    double old_angle = 0;
    unsigned long long start_iter;
    double cx = 0;
    double cy = 0;
    /// @brief Non-default constructor of Location
    /// @param r Robot instance
    void initialize(Robot& r);
    /// @brief Right absolute distance since start
    /// @return abs distance in inches
    double right_abs_dist();
    /// @brief Left absolute distance since start
    /// @return abs distance in inches
    double left_abs_dist();
    /// @brief Takes a raw angle and modifies it to bearing.
    /// @param deg degrees to format
    /// @return formates degrees in bearing from 0 - 359
    double normalize(double deg);
    /// @brief Calculates the amount to update coordinates
    /// @return Vector of updated coordinates
    std::vector<double> update();
    /// @brief Returns motor values depending on PID value
    /// @param goal goal we want to reach
    /// @return Vector of motor values. (0 is left, 1 is right)
    std::vector<double> updatePID(Waypoint& goal, CartesianLine& robot_line, CartesianLine& goal_line, bool turn = false);
    /// @brief Resets all encoders to 0
    void reset_all();
    /// @brief Checks if PID is still running...
    /// @return true (done) or false (not done)
    bool is_running();
    /// @brief Returns the absolute angle changed since begining of the program...
    /// @return Absolute angle in degrees...
    double get_angle_abs();
};

#endif // LOCATION_H