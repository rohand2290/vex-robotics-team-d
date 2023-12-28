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
#define LEFT_WHEELS_PORT_2 8 // Defines the 2nd port of the left wheels.
#define LEFT_WHEELS_PORT_3 7 // Defines the 3rd port of the left wheels.
// Right drive:
#define RIGHT_WHEELS_PORT_1 19 // Defines the 1st port of the right wheels.
#define RIGHT_WHEELS_PORT_2 18 // Defines the 2nd port of the right wheels.
#define RIGHT_WHEELS_PORT_3 17 // Defines the 3rd port of the right wheels.

// Intake mechanism:
#define INTAKE_PORT_LEFT 16 // Defines the 1st port of the left side of intake.
#define INTAKE_PORT_RIGHT 6 // Defines the 2nd port of the right side of intake.

// Catapult:
#define CATA_PORT 20 // Defines the port of the cata.
#define CATA_ENCODER 15 // Define the encoder port for cata
#define CATA_REST 90 // Defines the resting position of the cata...

// Pistons:
#define INTAKE_PISTON_PORT 'A'
#define WINGS_PORT 'H'

// IMU:
#define IMU_PORT 10

// ======== PREFERENCES: =========

// Driving style:
// uncomment the following line if smooth drive is not desired...
// #define SMOOTH_CONSTANT 20 // This constant effects the smoothness of the transition, the larger the less smooth.

// Motor speeds:
#define FLYWHEEL_SPEED 255 // the turret speed
#define MOTOR_PERCENT 1 // percentage (0-1) for max power to be used (0.75 means the maximum ever reached will be 75%)
#define TURN_PERCENT 1 // percentage (0-1) for max turn sensibility.

// ======= Auton: ================
// trackion wheels:
#define WHEEL_C 2.75 * PI // wheel circumference
#define TICKS_PER_REVOLUTION 300
// find difference from pivot point (point that doesnt move during rotation) and center odom.
#define PIVOT_P_TO_PERP_ODOM 0.56 // in
// encoder ports:
#define ENCODER_PORT_1 69 // left parralell
#define ENCODER_PORT_2 69 // right parralell
#define ENCODER_PORT_3 69 // center perpendicular
// Start Angle:
#define ANGLE_START 0 // bearing front
// PID Constants:
// power:
#define POWER_KP 10
#define POWER_KI 0
#define POWER_KD 2
#define POWER_ERROR_MAX 99999999
#define POWER_ERROR_MIN -99999999
// turn:
#define TURN_KP 0
#define TURN_KI 0
#define TURN_KD 0
#define TURN_ERROR_MAX 99999999
#define TURN_ERROR_MIN -99999999
// cata:
#define CATA_KP 1.9

// maximum allowed error:
#define MIN_ALLOWED_ERROR 10
#define MIN_ALLOWED_ERROR_TIME 50

// ======= Miscelanious: =========

// loop delay info: 
// If it is too large, it might decrease sensitivity and increase lag.
// this can also be used to change smoothness if smooth drive is enabled.
// If it is too little, it might starve the proccessor from energy with too much load.
#define OPCONTROL_LOOP_DELAY 20 // ms
#define AUTON_LOOP_DELAY 20 // ms
#define PI 3.141592653589793
#define ROBOT_WIDTH 3.54331 // in
// accuracy adjustments: the smaller, the more accurate. avoid making it 0
#define ACCURACY 0.01
#define G 9.80665

#endif // VARIABLES_H