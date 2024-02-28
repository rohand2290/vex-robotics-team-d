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
    left1  = new pros::Motor (LEFT_WHEELS_PORT_1, pros::E_MOTOR_GEAR_600);
    left2  = new pros::Motor (LEFT_WHEELS_PORT_2, pros::E_MOTOR_GEAR_600);
    left3  = new pros::Motor (LEFT_WHEELS_PORT_3, pros::E_MOTOR_GEAR_600);
    right1 = new pros::Motor (RIGHT_WHEELS_PORT_1, pros::E_MOTOR_GEAR_600, true);
    right2 = new pros::Motor (RIGHT_WHEELS_PORT_2, pros::E_MOTOR_GEAR_600, true);
    right3 = new pros::Motor (RIGHT_WHEELS_PORT_3, pros::E_MOTOR_GEAR_600, true);
    intake_left = new pros::Motor (INTAKE_PORT_LEFT, pros::E_MOTOR_GEAR_RED, pros::E_MOTOR_BRAKE_COAST);
    intake_right = new pros::Motor (INTAKE_PORT_RIGHT, pros::E_MOTOR_GEAR_RED, true);
    intake_right->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    cata = new pros::Motor(CATA_PORT, pros::E_MOTOR_GEAR_100, pros::E_MOTOR_BRAKE_COAST);
    wings = new pros::ADIDigitalOut (WING_1_PORT);
    wings_back = new pros::ADIDigitalOut (WING_2_PORT);
    pto_cata = new pros::ADIDigitalOut (PTO_CATA);
    pto_climb = new pros::ADIDigitalOut (PTO_CLIMB);
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
    delete pto_cata;
    delete pto_climb;
    delete encoder_left;
    delete encoder_right;
    delete encoder_center;
    delete encoder_cata;
    delete imu;
}