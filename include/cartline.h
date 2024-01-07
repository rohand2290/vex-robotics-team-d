#ifndef CARTLINE_H
#define CARTLINE_H

struct Point2D {
    double x = 0;
    double y = 0;
    Point2D(double X, double Y);
};

class CartesianLine {
private:
    int y_cof = 1;
public:
    double x;
    double y;
    double slope;
    /// @brief Gets the perpendicular slope of another slope
    /// @param s a slope
    /// @return the perpendicular counterpart
    static double get_perp(double s);
    /// @brief Default constructor...
    /// @param _slope slope wanted
    /// @param _x x coordinated to intersect
    /// @param _y y coordinated to intersect
    CartesianLine(double _slope, double _x, double _y);
    /// @brief Gets the slope of the line
    /// @return Slope of the line
    double get_slope();
    /// @brief Evaluates the Y given the X.
    /// @param X x coord
    /// @return the coresponding Y coordinate.
    double eval(double X);
    /// @brief Checks if point given is on line...
    /// @param X x coord
    /// @param Y y coord
    /// @return boolean if condition satisfied.
    bool is_on_line(double X, double Y);
    /// @brief Checks of point is above the line... (will return false is slope is UN)
    /// @param X x coord
    /// @param Y y coord
    /// @return boolean if condition satisfied.
    bool is_above(double X, double Y);
    /// @brief Checks of point is below the line... (will return false is slope is UN)
    /// @param X x coord
    /// @param Y y coord
    /// @return boolean if condition satisfied.
    bool is_bellow(double X, double Y);
};

class CartesianCircle {
private:
public:
    double h = 0;
    double k = 0;
    double r = 1;
    /// @brief sets the variables accordingly...
    /// @param H x coord of center
    /// @param K y coord of center
    /// @param R radius of circle
    CartesianCircle(double H, double K, double R);
    /// @brief Evaluates a given x value for a coresponding y 
    /// @param x desired x val
    /// @return 2 double values in an array for 2 y values...
    double* eval(double x); 
    /// @brief Checks if a given point is on the circumference... 
    /// @param x x coord
    /// @param y y coord
    /// @return if it intersects (true) or not (false)
    bool is_on_circum(double x, double y);
    /// @brief finds the derivtive at a given point.
    /// @param x point x
    /// @return nan if point is not on circle, and value if it is.
    double derivative_at(double x);
    /// @brief if one point and the origin were in the circumference of the circle, this computes that circles radius
    /// @param x x coord of point
    /// @param y y coord of point
    /// @param t modified heading of robot
    /// @return the radius of the tangent circle.
    double compute_radius(double x, double y, double t);
    /// @brief If 2 circles are created such that 1 is on the origin, and one is on (x, y); this function will compute all points that intersect the 2 circles...
    /// @param x x coord of point
    /// @param y y coord of point
    /// @param r radius of circle...
    /// @return vector of Point2D that intersect both circles.
    std::vector<Point2D> find_intersections(double x, double y, double r);
    /// @brief equal operator
    /// @param c one to set equal to
    void operator=(CartesianCircle& c);
};

#endif // CARTLINE_H