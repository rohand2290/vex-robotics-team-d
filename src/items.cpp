#include "depend.h"

void Items::initialize()  {
    master = new pros::Controller(pros::E_CONTROLLER_MASTER);
    left1  = new pros::Motor (LEFT_WHEELS_PORT_1, true);
    left2  = new pros::Motor (LEFT_WHEELS_PORT_2, true);
    left3  = new pros::Motor (LEFT_WHEELS_PORT_3);
    right1 = new pros::Motor (RIGHT_WHEELS_PORT_1);
    right2 = new pros::Motor (RIGHT_WHEELS_PORT_2);
    right3 = new pros::Motor (RIGHT_WHEELS_PORT_3, true);
    intake_left = new pros::Motor (INTAKE_PORT_LEFT);
    intake_right = new pros::Motor (INTAKE_PORT_RIGHT, true);
    turret = new pros::Motor(TURRET_PORT);
    puncher1 = new pros::ADIDigitalOut (PUNCHER_PORT_1, 0);
    puncher2 = new pros::ADIDigitalOut (PUNCHER_PORT_2, 0);
    initpos = 1;
    pto1 = new pros::ADIDigitalOut (PTO_PORT_1, initpos);
    pto2 = new pros::ADIDigitalOut (PTO_PORT_2, initpos);
    encoder_left = new pros::Rotation (ENCODER_PORT_1);
    encoder_right = new pros::Rotation (ENCODER_PORT_2);
    encoder_center = new pros::Rotation (ENCODER_PORT_3);
}

Items::~Items() {
    delete master;
    delete left1;
    delete left2;
    delete left3;
    delete right1;
    delete right2;
    delete right3;
    delete intake_left;
    delete intake_right;
    delete turret;
    delete puncher1;
    delete puncher2;
    delete pto1;
    delete pto2;
    delete encoder_left;
    delete encoder_right;
    delete encoder_center;
}