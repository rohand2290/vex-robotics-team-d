#include "depend.h"
#include "api.h"
#include "pros/motors.h"

void Items::initialize()  {
    wing_pos = false;
    wing_pos = false;
    intake_pos = false;
    autonmous = true;
    cata_state = false;
    master = new pros::Controller(pros::E_CONTROLLER_MASTER);
    left1  = new pros::Motor (LEFT_WHEELS_PORT_1, pros::E_MOTOR_GEAR_600, true);
    left2  = new pros::Motor (LEFT_WHEELS_PORT_2, pros::E_MOTOR_GEAR_600, true);
    left3  = new pros::Motor (LEFT_WHEELS_PORT_3, pros::E_MOTOR_GEAR_600, true);
    right1 = new pros::Motor (RIGHT_WHEELS_PORT_1, pros::E_MOTOR_GEAR_600);
    right2 = new pros::Motor (RIGHT_WHEELS_PORT_2, pros::E_MOTOR_GEAR_600);
    right3 = new pros::Motor (RIGHT_WHEELS_PORT_3, pros::E_MOTOR_GEAR_600);
    intake_left = new pros::Motor (INTAKE_PORT_LEFT, pros::E_MOTOR_GEAR_600, pros::E_MOTOR_BRAKE_COAST);
    intake_right = new pros::Motor (INTAKE_PORT_RIGHT, pros::E_MOTOR_GEAR_200, pros::E_MOTOR_BRAKE_COAST);
    intake_right->set_reversed(true);
    cata = new pros::Motor(CATA_PORT, pros::E_MOTOR_GEAR_100);
    wings = new pros::ADIDigitalOut (WING_1_PORT);
    wings_back = new pros::ADIDigitalOut (WING_2_PORT);
    pto = new pros::ADIDigitalOut (PTO);
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
    delete wings_back;
    delete pto;
    delete encoder_left;
    delete encoder_right;
    delete encoder_center;
    delete encoder_cata;
    delete imu;
}