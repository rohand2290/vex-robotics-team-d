#include "depend.h"

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
    for (int i = 0; i < N; ++i) v.setIndex(i, c_arr[i] * X);
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::div(double X) {
    VectorXD<N> v;
    for (int i = 0; i < N; ++i) c_arr[i] /= X;
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::add_by_vect(VectorXD<N> Vect) {
    VectorXD<N> v;
    for (int i = 0; i < N; ++i) v.setIndex(i, c_arr[i] + Vect.getIndex(i));
    return v;
}
template <int N>
VectorXD<N> VectorXD<N>::sub_by_vect(VectorXD<N> Vect) {
    VectorXD<N> v;
    for (int i = 0; i < N; ++i) v.setIndex(i, c_arr[i] - Vect.getIndex(i));
    return v;
}
template <int N>
int VectorXD<N>::size() const { return N; }
template<int N>
VectorXD<N> VectorXD<N>::rotate(double rad) {
    VectorXD<N> ret(c_arr[0], c_arr[1]);
    double arr2[] = {
        c_arr[0] * cos(rad) - c_arr[1] * sin(rad),
        c_arr[0] * sin(rad) + c_arr[1] * cos(rad)
    };
    ret.setIndex(0, arr2[0]);
    ret.setIndex(1, arr2[1]);
    return ret;
}