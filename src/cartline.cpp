#include "depend.h"
#include "cartline.h"

double CartesianLine::get_perp(double s)
{
    if (s * 0 != 0) return 0;
    return -1 / s;
}

CartesianLine::CartesianLine(double _slope, double _x, double _y)
{
    if (_slope * 0 != 0) y_cof = 0;
    slope = _slope;
    x = _x;
    y = _y;
}

double CartesianLine::get_slope() {
    return slope;
}

double CartesianLine::eval(double X) {
    return slope * (X - x) + y;
}

bool CartesianLine::is_on_line(double X, double Y) {
    return ARE_SAME(eval(X), Y);
};

bool CartesianLine::is_above(double X, double Y) {
    if (!y_cof) return false;
    double yVal = eval(X);
    return Y > yVal;
}

bool CartesianLine::is_bellow(double X, double Y) {
    if (!y_cof) return false;
    double yVal = eval(X);
    return Y < yVal;
}
