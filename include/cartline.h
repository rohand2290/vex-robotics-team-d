#ifndef CARTLINE_H
#define CARTLINE_H

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

#endif // CARTLINE_H