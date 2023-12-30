#ifndef FILTER_H
#define FILTER_H

class ComplementaryFilter {
private:
    double alpha;
public:
    /// @brief Default Constructor
    /// @param a value for Alpha [0, 1] (value will be cut of to fit range)
    ComplementaryFilter(double a);
    /// @brief Gets the value of alpha
    /// @return current alpha value
    double get_alpha();
    /// @brief Sets the value of alpha [0, 1] (value will be cut of to fit range)
    /// @param a new alpha value
    void set_alpha(double a);
    /// @brief Computes the complementary filter using the given sensor data.
    /// @param sensor1 Value from sensor 1 (alpha)
    /// @param sensor2 Value from sensor 2 (1 - alpha)
    /// @return Filtered value
    double compute(double sensor1, double sensor2);
};

class Kalman1VFilter {
private:
    double init = 0.001;    
    double errorEst = 0.001;
    double errorMea = 0.001;
    double oldErrorEst = 0.001;
    double est = 0.001;
    double oldEst = 0.001;
    double KG = 0.001;
public:
    /// @brief Constructor for a 1 Variable Kalman Filter
    /// @param EM Error measurement
    /// @param INIT Initial value
    Kalman1VFilter(double EM, double INIT);
    /// @brief Sets the error measurement...
    /// @param val new value to set
    void set_error_mea(double val);
    /// @brief Sets the initial value...
    /// @param val new value to set
    void set_init_val(double val);
    /// @brief Returns the Kalman filtered value of the estimate.
    /// @param sensor Sensor value to be filtered
    /// @return the new estimate.
    double compute(double sensor);
};

#endif // FILTER_H