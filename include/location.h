#ifndef LOCATION_H
#define LOCATION_H

#include "robot.h"

class Location {
private:
    // robot coordinates (later to be synced with Robot class)
    double *x;
    double *y;
    double *theta;
    // robot class:
    Robot robot;
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
public:
    void initialize(Robot& r);
    void update();
    
    double calc_theta_orient();
    VectorXD<2> local_offset();
    double avg_orient();
    
    VectorXD<2> global_offset(VectorXD<2> delta_dl);
};

#endif // LOCATION_H