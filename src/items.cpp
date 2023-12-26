#include "depend.h"
#include "api.h"

void Items::initialize()  {
    wing_pos = false;
    intake_pos = false;
    master = new pros::Controller(pros::E_CONTROLLER_MASTER);
    left1  = new pros::Motor (LEFT_WHEELS_PORT_1, true);
    left2  = new pros::Motor (LEFT_WHEELS_PORT_2, true);
    left3  = new pros::Motor (LEFT_WHEELS_PORT_3, true);
    right1 = new pros::Motor (RIGHT_WHEELS_PORT_1);
    right2 = new pros::Motor (RIGHT_WHEELS_PORT_2);
    right3 = new pros::Motor (RIGHT_WHEELS_PORT_3);
    intake_left = new pros::Motor (INTAKE_PORT_LEFT);
    intake_right = new pros::Motor (INTAKE_PORT_RIGHT, true);
    cata = new pros::Motor(CATA_PORT);
    wings = new pros::ADIDigitalOut (WINGS_PORT);
    intake_piston = new pros::ADIDigitalOut (INTAKE_PISTON_PORT);
    encoder_left = new pros::Rotation (ENCODER_PORT_1, true);
    encoder_right = new pros::Rotation (ENCODER_PORT_2);
    encoder_center = new pros::Rotation (ENCODER_PORT_3);
    encoder_cata = new pros::Rotation (CATA_ENCODER, true);
    imu = new pros::IMU (IMU_PORT);
}

void Items::stop() {
    left1->move(0);
    right1->move(0);
    right2->move(0);
    left2->move(0);
    left3->move(0);
    right3->move(0);
    intake_left->move(0);
    intake_right->move(0);
    cata->move(0);
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
    delete cata;
    delete wings;
    delete encoder_left;
    delete encoder_right;
    delete encoder_center;
    delete encoder_cata;
    delete intake_piston;
    delete imu;
}