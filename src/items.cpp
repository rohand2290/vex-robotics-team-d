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
    flywheel = new pros::Motor(FLYWHEEL_PORT);
    wings = new pros::ADIDigitalOut (WINGS_PORT);
    initpos = false;
    initpos2 = false;
    pto = new pros::ADIDigitalOut (PTO_PORT, initpos);
    encoder_left = new pros::Rotation (ENCODER_PORT_1);
    encoder_right = new pros::Rotation (ENCODER_PORT_2);
    encoder_center = new pros::Rotation (ENCODER_PORT_3);
}

void Items::stop() {
    left1->brake();
    left2->brake();
    left3->brake();
    right1->brake();
    right2->brake();
    right3->brake();
    intake_left->brake();
    intake_right->brake();
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
    delete flywheel;
    delete pto;
    delete wings;
    delete encoder_left;
    delete encoder_right;
    delete encoder_center;
}