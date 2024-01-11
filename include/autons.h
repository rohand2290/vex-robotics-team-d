#ifndef AUTONS_H
#define AUTONS_H

class PID {
private:
    double KP;
    double KI;
    double KD;
    double integral;
    double prev_error;
public:
    /// @brief Default constructor.. Enter PID values here
    /// @param kp Proportion constant
    /// @param ki Integral constant
    /// @param kd Derivative constant
    PID(double kp, double ki, double kd);
    /// @brief Default Constructor.
    PID();
    /// @brief Non-default Constructor..
    /// @param kp Proportion constant
    /// @param ki Integral constant
    /// @param kd Derivative constant
    void initialize(double kp, double ki, double kd);
    /// @brief Computes the PID for a given error value
    /// @param error error value from sensor
    /// @return PID computed power
    double update(double error);
    /// @brief Resets the internal values... Constants are left same.
    void reset();
};


#endif