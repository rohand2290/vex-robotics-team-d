#ifndef LOCATION_H
#define LOCATION_H

#include "depend.h"

class IMULocation {
private:
    Robot& robot;
    Items& items;
    double x;
    double y;

    // helper vectors:
    VectorXD<2> prev_vel;
    VectorXD<2> prev_dis;
public:
    /// @brief Default constructor... Uses purely IMU based navigation.
    /// @param r Robot instance.
    IMULocation(Robot& r);
    /// @brief Computes the current coordinates...
    void compute();
    // /// @brief Returns current Dispacement in X-axis
    // /// @return Displacement in meters...
    // double getX();
    // /// @brief Returns current Dispacement in Y-axis
    // /// @return Displacement in meters...
    // double getY();
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
    double prev_error = 0;
    double integral = 0;
    double error_l = 0;
    double prev_error_l = 0;
    double integral_l = 0;
    // PID timer:
    int timer = 0;
public:
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
    /// @brief Proportion of PID
    /// @param error current error
    /// @param isturn calculating turn or power PID?
    /// @return Proportion
    double P(double error, bool isturn);
    /// @brief Integral of PID
    /// @param error current error
    /// @param integral reference to integral
    /// @param goal goal we want to reach
    /// @param isturn calculating turn or power PID?
    /// @return Integral
    double I(double error, double& integral, Waypoint& goal, bool isturn);
    /// @brief Derivative of PID
    /// @param prev_error old caches error
    /// @param error curent error
    /// @param isturn calculating turn or power PID?
    /// @return Derivative
    double D(double& prev_error, double error, bool isturn);
    /// @brief Calculates the PID number.
    /// @param error current error
    /// @param integral reference to integral
    /// @param prev_error old cached error
    /// @param goal goal we want to reach
    /// @param isturn calculating turn or power PID?
    /// @return PID number
    double pid(double error, double& integral, double& prev_error, Waypoint& goal, bool isturn);
    /// @brief Returns motor values depending on PID value
    /// @param goal goal we want to reach
    /// @return Vector of motor values. (0 is left, 1 is right)
    std::vector<double> updatePID(Waypoint& goal);
    /// @brief Resets all encoders to 0
    void reset_all();
    /// @brief Checks if PID is still running...
    /// @return true (done) or false (not done)
    bool is_running();
};

#endif // LOCATION_H