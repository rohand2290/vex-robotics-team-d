#include "depend.h"
#include "api.h"
#include "robot.h"

static void set_in(int a, Items& items) {
    items.intake_left->move(a);
    items.intake_right->move(a);
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
    // logic: arc-length = WHEEL_C
    //      : radius = perpendicular sensor - point of pivoting
    //      : theta / arc-length = 2 * pi / 2 * pi * radius
    //      => theta =  arc-length / radius RADIANS
    // double mag = (center_abs_dist());
    // double theta = mag / PIVOT_P_TO_PERP_ODOM;
    return rad ? degrees_to_radians(items.imu->get_heading()) : items.imu->get_heading();
}

void Robot::initialize(Items &i)
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
    items.left1->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    items.left2->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    items.left3->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    items.right1->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    items.right2->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    items.right3->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    items.imu->reset(true);
    items.imu->tare();
    items.imu->set_heading(0);
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

void Robot::set_both_sides(int right, int left)
{
    if (items.autonmous) {
        // sensitive for auton:
        right *= 120000 / 127;
        left *= 120000 / 127;
        items.right1->move_voltage(right);
        items.left1->move_voltage(left);
        items.left2->move_voltage(left);
        items.right2->move_voltage(right);
        items.right3->move_voltage(right);
        items.left3->move_voltage(left);
    } else {
        // smooth for drive:
        items.right1->move(right);
        items.left1->move(left);
        items.left2->move(left);
        items.right2->move(right);
        items.right3->move(right);
        items.left3->move(left);
    }
}

void Robot::set_speed_chassis(int y, int x)
{
    int left = y - x;
    int right = y + x;
    set_both_sides(right, left);
}

void Robot::set_intake(int analog1, int analog2, int pist)
{
    if (intake_state == 0) {
        if (analog1) {
            set_in(-255, items);
            intake_state = 1;
        } else if (analog2) {
            set_in(255, items);
            intake_state = -1;
        }
    } else if (intake_state == 1) {
        if (analog1) {
            set_in(0, items);
            intake_state = 0;
        } else if (analog2) {
            set_in(255, items);
            intake_state = -1;
        }
    } else if (intake_state == -1) {
        if (analog1) {
            set_in(-255, items);
            intake_state = 1;
        } else if (analog2) {
            set_in(0, items);
            intake_state = 0;
        }
    }
}

void Robot::set_cata(int analog) {
    if (!analog) {
        // P Stuff:
        // double power = CATA_KP * (CATA_REST - get_cata_position());
        // items.cata->move(ABS(power));
        items.cata->move(0);
    } else {
        items.cata->move_voltage(120000);
    }
}

void Robot::set_wings(int stick)
{
    // auto now = std::chrono::high_resolution_clock::now();
    // if (items.intake_pos && stick) {
    //     items.intake_piston->set_value(0);
    //     int dur = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - time).count();
    //     if (dur > 500) {
    //         items.wing_pos = 1;
    //         items.wings->set_value(items.wing_pos);
    //     };
    // }
    if (stick) items.wing_pos = !items.wing_pos;
    items.wings->set_value(items.wing_pos);
}

void Robot::set_blocker(int analog) {
    if (analog) items.intake_pos = !items.intake_pos;
    
    if (items.intake_pos) {
        items.lift1->set_value(1);
        items.lift2->set_value(1);
    } else {
        items.lift1->set_value(0);
        items.lift2->set_value(0);
    }
}

double Robot::radians_to_degrees(double radians)
{
    return (radians * 180) / PI;
}

double Robot::degrees_to_radians(double degrees)
{
    return (degrees * PI) / 180;
}
