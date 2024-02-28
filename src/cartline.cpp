#include "depend.h"
#include "cartline.h"

Point2D::Point2D(double X, double Y): x(X), y(Y) {};

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

CartesianCircle::CartesianCircle(double H, double K, double R): h(H), k(K), r(R) {};

double* CartesianCircle::eval(double x) {
    double c = sqrt(r*r - x*x);
    double tor[] = {c, -c};
    return nullptr;
}

double CartesianCircle::derivative_at(double x) {
    double y = eval(x)[0];
    return (x/y) * 0 == 0 ? x / y : 200000;
}

bool CartesianCircle::is_on_circum(double X, double Y) {
    return (X-h)*(X-h) + (Y-k)*(Y-k) == r*r;
}

double CartesianCircle::compute_radius(double x, double y, double t) {
    return sqrt(x*x + y*y) / ABS(2 * sin(t/2));
}

std::vector<Point2D> CartesianCircle::find_intersections(double x, double y, double r) {
    std::vector<Point2D> tor(4, (Point2D){0, 0});
    double c = (x*x + y*y) / (2*y);
    double m = x / y;
    {
        double d = sqrt(abs(4*c*c*m*m - 4*(1 + m*m)));
        double x1 = (2*c*m + d) / (2 * (1 + m*m));
        double x2 = (2*c*m - d) / (2 * (1 + m*m));
        tor[0].x = x1;
        tor[1].x = x1;
        tor[2].x = x2;
        tor[3].x = x2;
    }
    {
        double k1 = sqrt(r*r - tor[0].x*tor[0].x);
        double k2 = sqrt(r*r - tor[2].x*tor[2].x);
        tor[0].y = k1;
        tor[1].y = -k1;
        tor[2].y = k2;
        tor[3].y = -k2;
    }
    std::vector<Point2D> res;
    for (Point2D i : tor) {
        CartesianCircle circ(i.x, i.y, r);
        if (i.x * 0 != 0 && i.y * 0 != 0 && circ.is_on_circum(x, y)) res.push_back(i);
    }
    return res;
}

void CartesianCircle::operator=(CartesianCircle& c) {
    h = c.h;
    k = c.k;
    r = c.r;
}
