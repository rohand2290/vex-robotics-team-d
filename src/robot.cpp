#include "depend.h"
#include "api.h"
#include "robot.h"

static void set_in(int a, Items& items) {
    if (a > 0) {
        items.intake_left->move_velocity(100);
        items.intake_right->move_velocity(100);
    } else if (a < 0) {
        items.intake_left->move_velocity(-100);
        items.intake_right->move_velocity(-100);
    }
    else {
        items.intake_left->brake();
        items.intake_right->brake();
    }
}

double Robot::get_cata_position() {
    return ((int)items.encoder_cata->get_position()) / 100.0;
}

void Robot::set_coast() {
    items.right1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.right2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.right3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    items.left1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.left2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.left3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Robot::set_hold() {
    items.right1->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    items.right2->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    items.right3->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    items.left1->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    items.left2->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    items.left3->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

double Robot::get_abs_angle(bool rad) {
    double a = items.imu->get_heading();
    return rad ? degrees_to_radians(a) : a;
}

void Robot::initialize(Items& i)
{
    items = i;
    x = 0;
    y = 0;
    theta = ANGLE_START; // change depending on start pos
    items.encoder_left->reset_position();
    items.encoder_right->reset_position();
    items.encoder_center->reset_position();
    items.encoder_cata->reset_position();
    items.cata->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.right1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.right2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.right3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.left1->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.left2->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.left3->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    items.left1->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    items.left2->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    items.left3->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    items.right1->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    items.right2->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    items.right3->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    items.imu->reset(true);
    items.imu->tare();
    items.imu->set_heading(0);
    brake_pid_left.initialize(BREAK_KP, 0, 0);
    brake_pid_right.initialize(BREAK_KP, 0, 0);
}

Robot::~Robot() {}

void Robot::set_right_side(int analog)
{
    items.left1->move(analog);
    items.left2->move(analog);
    items.left3->move(analog);
}

void Robot::set_left_side(int analog)
{
    items.right1->move(analog);
    items.right2->move(analog);
    items.right3->move(analog);
}

void Robot::set_both_sides(int right, int left, bool slew)
{
    if (items.autonmous) {
        // sensitive for auton:
        right *= 12000.0 / 127;
        left *= 12000.0 / 127;
        /*if (slew) {
            if (abs(right - prev_volt_right) > SLEW_RATE && abs(right) < 12000) {
                if (right < prev_volt_right) right = prev_volt_right - SLEW_RATE;
                else right = prev_volt_right + SLEW_RATE;
            }
            if (abs(left - prev_volt_left) > SLEW_RATE && abs(left) < 12000) {
                if (left < prev_volt_left) left = prev_volt_left - SLEW_RATE;
                else left = prev_volt_left + SLEW_RATE;
            }
            prev_volt_left = left;
            prev_volt_right = right;
        }
        else {
            prev_volt_left = 0;
            prev_volt_right = 0;
        }*/

        items.right1->move_voltage(right);
        items.left1->move_voltage(left);
        items.left2->move_voltage(left);
        items.right2->move_voltage(right);
        items.right3->move_voltage(right);
        items.left3->move_voltage(left);
    }
    else {
        // smooth for drive:
        items.right1->move(right);
        items.left1->move(left);
        items.left2->move(left);
        items.right2->move(right);
        items.right3->move(right);
        items.left3->move(left);
    }
}

static double get_avg_dis(Items& items) {
    double normal = items.right1->get_position() + items.right2->get_position() + items.right3->get_position();
    double normal2 = items.left1->get_position() + items.left2->get_position() + items.left3->get_position();
    return (normal + normal2) / 2;
}

void Robot::set_speed_chassis(int y, int x)
{
    if (abs(y) > JOYSTICK_DEADZONE || abs(x) > JOYSTICK_DEADZONE) {
        int left = y - x;
        int right = y + x;
        set_both_sides(right, left);
    }
    else {
        // double error = current_val - get_avg_dis(items);
        // int power = brake_pid.update(error);
        // set_both_sides(power, power);
        set_both_sides(0, 0);
    }
}

void Robot::set_intake(int analog1, int analog2, int pist)
{
    // if (intake_state == 0) {
    //     if (analog1) {
    //         set_in(-255, items);
    //         intake_state = 1;
    //     } else if (analog2) {
    //         set_in(255, items);
    //         intake_state = -1;
    //     }
    // } else if (intake_state == 1) {
    //     if (analog1) {
    //         set_in(0, items);
    //         intake_state = 0;
    //     } else if (analog2) {
    //         set_in(255, items);
    //         intake_state = -1;
    //     }
    // } else if (intake_state == -1) {
    //     if (analog1) {
    //         set_in(-255, items);
    //         intake_state = 1;
    //     } else if (analog2) {
    //         set_in(0, items);
    //         intake_state = 0;
    //     }
    // }
    if (analog1) {
        // set intake to high
        set_in(-127, items);
    }
    if (analog2) {
        // set intake to high
        set_in(127, items);
    }
    if (!analog1 && !analog2) {
        // set intake to low
        set_in(0, items);
    }
}

void Robot::set_cata(int analog, int analog2) {
    if (!analog) {
        // P Stuff:
        //double power = CATA_KP * (CATA_REST - get_cata_position());
        //items.cata->move(power);
        // items.cata->move(0);

        // disable cata pto
        items.pto_cata->set_value(0);
        //set_in(0, items);
    }
    else {
        if (items.intake_pos) {
            items.intake_pos = 0;
            items.pto_climb->set_value(0);
        }
        items.pto_cata->set_value(1);
        // items.cata->move_voltage(120000);
        items.intake_left->move_velocity(50);
        items.intake_right->move_velocity(50);
    }
}

void Robot::set_wings(int stick)
{
    if (stick) items.wing_pos = !items.wing_pos;
    items.wings->set_value(items.wing_pos);
}

void Robot::set_wings_back(int stick)
{
    if (stick) items.wing_back_pos = !items.wing_back_pos;
    items.wings_back->set_value(items.wing_back_pos);
}

void Robot::set_blocker(int analog, int lock) {
    if (analog) items.intake_pos = !items.intake_pos;

    if (items.intake_pos) {
        items.pto_climb->set_value(1);
        if (lock) lock_state = !lock_state;
        if (lock_state) {
            items.intake_right->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            items.intake_right->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
        else {
            items.intake_right->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            items.intake_right->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }
    }
    else {
        items.pto_climb->set_value(0);
        lock_state = 0;
        items.intake_right->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        items.intake_right->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
}

double Robot::radians_to_degrees(double radians)
{
    return (radians * 180) / PI;
}

double Robot::degrees_to_radians(double degrees)
{
    return degrees * 0.0174533;
}

void Robot::brake() {
    set_hold();
    items.right1->brake();
    items.right2->brake();
    items.right3->brake();
    items.left1->brake();
    items.left2->brake();
    items.left3->brake();
    double in;
    do {
        in = sqrt(
            items.imu->get_accel().z * items.imu->get_accel().z +
            items.imu->get_accel().y * items.imu->get_accel().y
        );
        pros::delay(1);
    } while (abs(in) < 0.001);
    set_coast();
}

void Robot::run_cata_x_times(int x) {
    double prev_pos = get_cata_position();
    int count = 0, temp = false;
    items.cata->move_voltage(120000);
    do {
        pros::delay(20);
        double change = get_cata_position() - prev_pos;
        if (change <= 0 && !temp) {
            ++count;
            temp = true;
        }
        if (temp && change > 0) temp = false;
        prev_pos = get_cata_position();
    } while (count < x);
    items.cata->brake();
}

void Robot::break_absolute() {
    items.left1->move(0);
    items.left2->move(0);
    items.left3->move(0);
    items.right1->move(0);
    items.right2->move(0);
    items.right3->move(0);
}