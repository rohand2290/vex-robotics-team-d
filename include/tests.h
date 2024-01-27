#ifndef TESTS_H
#define TESTS_H

#include "depend.h"
#include "api.h"

class Tester {
private:
    Robot& robot;
    Items& items;
    static double get_avg(std::vector<double> a) {
        double m = 0;
        int i = 0;
        for (; i < a.size(); ++i) {
            m += a[i];
        }
        return m / i;
    }
public:
    /// @brief Default constructor of the Tests class...
    /// @param r Robot instance
    /// @param i Items instance
    Tester(Robot& r, Items& i): robot(r), items(i) {};
    
    /// @brief Tests the Motors on the chassis. Prints results on the remote. Make sure robot is off the ground.
    void test_chassis() {
        pros::lcd::clear();
        items.master->clear();
        items.master->print(0, 0, "Lift robot up");
        for (int i = 5; i >= 0; --i) {
            items.master->print(1, 0, "Starting in: %i", i);
            pros::delay(1000);
        }
        std::vector<std::int8_t> r = {RIGHT_WHEELS_PORT_1, RIGHT_WHEELS_PORT_2, RIGHT_WHEELS_PORT_3};
        std::vector<std::int8_t> l = {LEFT_WHEELS_PORT_1, LEFT_WHEELS_PORT_2, LEFT_WHEELS_PORT_3};
        pros::MotorGroup right(r);
        pros::MotorGroup left(l);
        right.move_voltage(12000);
        left.move_voltage(12000);
        bool speedr = false;
        double max_r = 0;
        bool speedl = false;
        double max_l = 0;
        bool equal = false;
        for (int i = 0; i < 10000; ++i) {
            double _r = get_avg(right.get_actual_velocities());
            double _l = get_avg(left.get_actual_velocities());
            if (ARE_SAME(ABS(_r), 600)) {
                speedr = true;
                if (_r > max_r) max_r = _r;
            }
            if (ARE_SAME(ABS(_l), 600)) {
                speedl = true;
                if (_l > max_l) max_l = _l;
            }
            if (ARE_SAME(_r, _l)) {
                equal = true;
            }
            pros::delay(1);

            if (items.master->get_digital(pros::E_CONTROLLER_DIGITAL_A)) break;
        }
        items.stop();

        items.master->clear();
        pros::lcd::print(0, "R RPM: %i %s", (int)max_r, speedr ? "pass" : "fail");
        pros::lcd::print(1, "L RPM: %i %s", (int)max_l, speedl ? "pass" : "fail");
        pros::lcd::print(2, "Speed E: %s", equal ? "pass" : "fail");
        TERMINATE();
    }
    /// @brief Gets the raw coordinates for driving forever...
    void get_raw_coordinates() {
        items.master->clear();
        int state = 0;
        Location maping;
        maping.initialize(robot);
        while (true) {            
            double x;
            double y;
            while (!state) {
                maping.update();
                if (items.master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) state = !state;

                x = maping.cx;
                y = maping.cy;
                pros::delay(5);           
            }
            maping.reset_all();
            items.master->print(0, 0, "%f,%f", x, y);

            while (!state) {
                if (items.master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) state = !state;
                pros::delay(5);
            }

            items.master->clear_line(0);
        }
    }
    /// @brief Gets the cata position to help with cata PID
    void get_cata_pos() {
        double pos;
        while (true) {
            pos = robot.get_cata_position();
            if (items.master->get_digital(pros::E_CONTROLLER_DIGITAL_A))
                items.master->print(0, 0, "%f", pos);
            pros::delay(5);
        }
    }
};

#endif // TESTS_H