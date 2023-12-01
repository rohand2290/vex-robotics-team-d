#ifndef LOCATION_H
#define LOCATION_H

#include "robot.h"

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
	double old_c = 0;
	// seperate trackers for theta:
	double old_t = 0;
	double new_t = 0;
	// change variables. Show change from previous set of data.
	double rel_l = 0;
	double rel_r = 0;
	double rel_c = 0;
    // PID:
    double error = 0;
    double prev_error = 0;
    double integral = 0;
    double error_turn = 0;
    double prev_error_turn = 0;
    double integral_turn = 0;
public:
    void initialize(Robot& r);
    int normalize(double deg); // returns an angle back from 0 - 360 bearing north (north is its initial position)
    std::vector<double> update(); // returns a double array
    
    // ODOM:
    double calc_theta_orient();
    VectorXD<2> local_offset();
    double avg_orient();
    VectorXD<2> global_offset(VectorXD<2> delta_dl);
    // PID:
    double P(double error, bool isturn);
    double I(double error, double& integral, Waypoint& goal, bool isturn);
    double D(double& prev_error, double error, bool isturn);
    double PID(double error, double& integral, double& prev_error, Waypoint& goal, bool isturn);
    std::vector<double> updatePID(Waypoint& goal);
};

#endif // LOCATION_H