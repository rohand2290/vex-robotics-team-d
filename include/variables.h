/*
 * Variables.h
 *
 * This file contains all constants that can change the behaviour of the code. 
 * Use this file to fine tune or change constants before compilation.
*/
#ifndef VARIABLES_H
#define VARIABLES_H

// ======== PORTS: ===========

// Left drive:
#define LEFT_WHEELS_PORT_1 9 // Defines the 1st port of the left wheels.
#define LEFT_WHEELS_PORT_2 20 // Defines the 2nd port of the left wheels.
#define LEFT_WHEELS_PORT_3 10 // Defines the 3rd port of the left wheels.
// Right drive:
#define RIGHT_WHEELS_PORT_1 11 // Defines the 1st port of the right wheels.
#define RIGHT_WHEELS_PORT_2 1 // Defines the 2nd port of the right wheels.
#define RIGHT_WHEELS_PORT_3 2 // Defines the 3rd port of the right wheels.

// Intake mechanism:
#define INTAKE_PORT_LEFT 8 // Defines the 1st port of the left side of intake.
#define INTAKE_PORT_RIGHT 4 // Defines the 2nd port of the right side of intake.

// Turret:
#define FLYWHEEL_PORT 69 // Defines the port of the turret motor.

// Puncher:
#define PTO_PORT 'B'
#define WINGS_PORT 'C'

// ======== PREFERENCES: =========

// Driving style:
// uncomment the following line if smooth drive is not desired...
#define SMOOTH_CONSTANT 20 // This constant effects the smoothness of the transition, the larger the less smooth.

// Motor speeds:
#define FLYWHEEL_SPEED 255 // the turret speed
#define MOTOR_PERCENT 1 // percentage (0-1) for max power to be used (0.75 means the maximum ever reached will be 75%)
#define TURN_PERCENT 1 // percentage (0-1) for max turn sensibility.

// ======= Auton: ================
// trackion wheels:
#define WHEEL_C 2.75 * PI // wheel circumference
// find difference from pivot point (point that doesnt move during rotation) and center odom.
#define PIVOT_P_TO_PERP_ODOM 0.32 // in
// encoder ports:
#define ENCODER_PORT_1 5 // left parralell
#define ENCODER_PORT_2 3 // right parralell
#define ENCODER_PORT_3 6 // center perpendicular
// Start Angle:
#define ANGLE_START 0 // bearing front
// PID Constants:
// power:
#define POWER_KP 7.85
#define POWER_KI 1
#define POWER_KD 2.26
#define POWER_ERROR_MAX 100
#define POWER_ERROR_MIN -100
// turn:
#define TURN_KP 2.85
#define TURN_KI 1
#define TURN_KD 1
#define TURN_ERROR_MAX 69
#define TURN_ERROR_MIN -69

// ======= Miscelanious: =========

// loop delay info: 
// If it is too large, it might decrease sensitivity and increase lag.
// this can also be used to change smoothness if smooth drive is enabled.
// If it is too little, it might starve the proccessor from energy with too much load.
#define OPCONTROL_LOOP_DELAY 1 // ms
#define AUTON_LOOP_DELAY 1 // ms
#define PI 3.141592653589793
#define ROBOT_WIDTH 3.54331 // in
// accuracy adjustments: the smaller, the more accurate. avoid making it 0
#define ACCURACY 0.00001
// where we start:
// #define START_RED_ALLY

#endif // VARIABLES_H