#include "depend.h"
#include "robot.h"

double Robot::right_abs_dist()
{ // in terms of inches
    return (items.encoder_right->get_position());
}

double Robot::left_abs_dist()
{ // in terms of inches
    return (items.encoder_left->get_position());
}

double Robot::center_abs_dist()
{ // in terms of inches
    return (items.encoder_center->get_position());
}

void Robot::initialize(Items &i)
{
    items = i;
    x = 0;
    y = 0;
    theta = 0;
    items.encoder_left->reset_position();
    items.encoder_right->reset_position();
    items.encoder_center->reset_position();
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

// here, y represents power and x represents turn, since this is arcade drive style.
void Robot::set_speed_chassis(int y, int x, long long line, int &speedr, int &speedl)
{
    int left = (y - (x * TURN_PERCENT)) * MOTOR_PERCENT;
    int right = (y + (x * TURN_PERCENT)) * MOTOR_PERCENT;
// set the wheels...
#ifdef SMOOTH_CONSTANT
    // left side:
    if (speedl >= left)
        speedl -= SMOOTH_CONSTANT;
    else if (speedl < left)
        speedl += SMOOTH_CONSTANT;
    // right side:
    if (speedr >= right)
        speedr -= SMOOTH_CONSTANT;
    else if (speedr < right)
        speedr += SMOOTH_CONSTANT;

    set_left_side(speedl);
    set_right_side(speedr);
#else
    set_left_side(left);
    set_right_side(right);
#endif
}

void Robot::set_intake(int analog1, int analog2)
{
    if (analog1)
    {
        items.intake_left->move(255);
        items.intake_right->move(255);
    }
    else if (analog2)
    {
        items.intake_left->move(255);
        items.intake_right->move(255);
    } else {
        items.intake_left->move(0);
        items.intake_right->move(0);
    }
}

void Robot::set_turret(int up, int down)
{
    if (up)
    {
        items.turret->move(TURRET_SPEED);
    }
    else
    {
        items.turret->move(0);
    }
    if (down)
    {
        items.turret->move(-TURRET_SPEED);
    }
    else
    {
        items.turret->move(0);
    }
}

void Robot::set_puncher(int analog)
{
    items.puncher1->set_value(analog);
    items.puncher2->set_value(analog);
}

double Robot::radians_to_degrees(double radians)
{
    return (radians * 180) / PI;
}

double Robot::degrees_to_radians(double degrees)
{
    return (degrees * PI) / 180;
}
