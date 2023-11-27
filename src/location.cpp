#include "depend.h"
#include "vectorxd.h"

template <int N>
VectorXD<N>::VectorXD(double x, double y) {
    if (N != 2) {
        pros::lcd::print(1, "ERROR, your vector is %i rows, but to use this constructor it must be 2,", size());
        TERMINATE();
    }
    c_arr[0] = x;
    c_arr[1] = y;
}
template <int N>
VectorXD<N>::VectorXD() {   
    for (int i = 0; i < N; ++i) {
        c_arr[i] = 0;
    }
}
template <int N>
double VectorXD<N>::getIndex(int i) {
    if (i >= size()) {
        pros::lcd::print(1, "ERROR, index is out of bounds [%i out of %i]", i, size());
        TERMINATE();
    }
    return c_arr[i];
}
template <int N>
void VectorXD<N>::setIndex(int i, double val) {
    if (i >= size()) {
        pros::lcd::print(1, "ERROR, index is out of bounds [%i out of %i]", i, size());
        TERMINATE();
    }
    c_arr[i] = val;
}
template <int N>
VectorXD<N> VectorXD<N>::mult(double X) {
    VectorXD<N> v;
    for (int i = 0; i < N; ++i) c_arr[i] *= X;
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::div(double X) {
    VectorXD<N> v();
    for (int i = 0; i < N; ++i) c_arr[i] /= X;
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::add_by_vect(VectorXD<N> Vect) {
    VectorXD<N> v();
    for (int i = 0; i < N; ++i) v[i] = c_arr[i] + Vect[i];
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::sub_by_vect(VectorXD<N> Vect) {
    VectorXD<N> v();
    for (int i = 0; i < N; ++i) v[i] = c_arr[i] - Vect[i];
    return v;
}
template <int N>
int VectorXD<N>::size() const { return N; }






void Location::initialize(Robot& r) {
    robot = r;
    x = &r.x;
    y = &r.y;
}
double Location::calc_theta_orient()
{
    return old_t + (rel_l - rel_r) / (ROBOT_WIDTH);
}
VectorXD<2> Location::local_offset()
{
    VectorXD<2> v;
    if (new_t - old_t == 0) {
        v.setIndex(0, rel_c);
        v.setIndex(1, rel_r);
    }
    else {
        v.setIndex(0, rel_c / (new_t - old_t) + 9.251969);
        v.setIndex(1, rel_r / (new_t - old_t) + (ROBOT_WIDTH / 2));
        v.mult(2 * sin(old_t / 2));
    }
    return v;
}
double Location::avg_orient() { 
    return old_t + (new_t - old_t) / 2; 
}
VectorXD<2> Location::global_offset(VectorXD<2> delta_dl) {
    double r = sqrt((*x) * (*x) + (*y) * (*y));
    double theta = atan2(*y, *x) - avg_orient();
    
    double x = r * cos(theta);
    double y = r * sin(theta);
    
    VectorXD<2> vect(x, y);
    return vect;
}
void Location::update() {
    // update variables:
    rel_l = robot.left_abs_dist() - old_l;
	rel_r = robot.right_abs_dist() - old_r;
	rel_c = robot.center_abs_dist() - old_c;
	new_t = calc_theta_orient();

    // calculate and update coordinates:
    VectorXD<2> vect = local_offset();
    double avgOr = avg_orient();
    VectorXD<2> change = global_offset(vect);
    *x += change.getIndex(0);
    *y += change.getIndex(0);

    // save new vars to cache:
    old_l = robot.left_abs_dist();
	old_r = robot.right_abs_dist();
	old_c = robot.center_abs_dist();
	old_t = new_t;
}