#include "depend.h"
#include "robot.h"

double Robot::right_abs_dist()
{ // in terms of inches
    double normal = items.right1->get_position() + items.right2->get_position() + items.right3->get_position();
    return normal / 3;
}

double Robot::left_abs_dist()
{ // in terms of inches
    double normal = items.left1->get_position() + items.left2->get_position() + items.left3->get_position();
    return normal / 3;
}

double Robot::center_abs_dist()
{ // in terms of inches
    int normal = (int)items.encoder_center->get_position();
    return (normal / 360000.0) * WHEEL_C;
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
    items.imu->reset(true);
    items.imu->tare();
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
    items.right1->move(right);
    items.left1->move(left);
    items.left2->move(left);
    items.right2->move(right);
    items.right3->move(right);
    items.left3->move(left);
}

void Robot::set_speed_chassis(int y, int x, long long line, int &speedr, int &speedl)
{
    int left = (y - (x * TURN_PERCENT)) * MOTOR_PERCENT;
    int right = (y + (x * TURN_PERCENT)) * MOTOR_PERCENT;
    // set the wheels...
    // #ifdef SMOOTH_CONSTANT
    //     // left side:
    //     if (speedl >= left)
    //         speedl -= SMOOTH_CONSTANT;
    //     else if (speedl < left)
    //         speedl += SMOOTH_CONSTANT;
    //     // right side:
    //     if (speedr >= right)
    //         speedr -= SMOOTH_CONSTANT;
    //     else if (speedr < right)
    //         speedr += SMOOTH_CONSTANT;

    //     set_left_side(speedl);
    //     set_right_side(speedr);
    // #else
    if (items.pto_pos) {
        items.left2->set_reversed(0);
        items.right2->set_reversed(1);
    } else {
        items.left2->set_reversed(1);
        items.right2->set_reversed(0);
    }
    set_both_sides(right, left);
    // #endif
}

static void set_in(int a, Items& items) {
    items.intake_left->move(a);
    items.intake_right->move(a);
}
void Robot::set_intake(int analog1, int analog2, int pist)
{
    if (analog1) {
        power = -255;
        if (temp1) power = 0;
        temp1 = !temp1;
    }
    if (analog2) {
        power = 255;
        if (temp2) power = 0;
        temp2 = !temp2;
    }
    set_in(power, items);
    if (pist) items.intake_pos = !items.intake_pos;
    items.intake_piston->set_value(items.intake_pos);
}

void Robot::set_flywheel(int stick)
{
    if (stick) items.flywheel_pos = !items.flywheel_pos;
    items.flywheel->move(items.flywheel_pos ? 255 : 0);
}

void Robot::set_wings(int stick)
{
    if (stick) items.wing_pos = !items.wing_pos;
    items.wings->set_value(items.wing_pos);
}

void Robot::set_pto(int input)
{
    items.pto->set_value(input);
}

double Robot::radians_to_degrees(double radians)
{
    return (radians * 180) / PI;
}

double Robot::degrees_to_radians(double degrees)
{
    return (degrees * PI) / 180;
}
