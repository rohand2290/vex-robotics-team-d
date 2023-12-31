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
#define INTAKE_PISTON_PORT 69
#define WINGS_PORT 'H'
#define LIFT_1_PORT 'A'
#define LIFT_2_PORT 'B'

// IMU:
#define IMU_PORT 10

// ======== PREFERENCES: =========

// Motor speeds:
#define FLYWHEEL_SPEED 255 // the turret speed

// ============== Auton: ================
// trackion wheels:
#define WHEEL_C 8.63937979737 // wheel circumference
#define TICKS_PER_REVOLUTION 300
#define APPROACH_SPEED 550
// find difference from pivot point (point that doesnt move during rotation) and center odom.
#define PIVOT_P_TO_PERP_ODOM 0.56 // in
// encoder ports:
#define ENCODER_PORT_1 69 // left parralell
#define ENCODER_PORT_2 69 // right parralell
#define ENCODER_PORT_3 69 // center perpendicular
// Start Angle:
#define ANGLE_START 0 // bearing front
// ============================ PID Constants: ======================
// power:
#define POWER_KP 0.75
#define POWER_KI 1
#define POWER_KD 1.2
#define POWER_ERROR_MAX 99999999
#define POWER_ERROR_MIN -99999999
// turn:
#define TURN_KP 0.18
#define TURN_KI 0
#define TURN_KD 1
#define TURN_ERROR_MAX 99999999
#define TURN_ERROR_MIN -99999999
// cata:
#define CATA_KP 0.5
// ======================= ODOM: ========================
#define ERROR_MEASUREMENT 0.05

// maximum allowed error:
#define MIN_ALLOWED_ERROR 1.7
#define MIN_ALLOWED_ERROR_DEG 10
#define MIN_ALLOWED_ERROR_TIME 70
#define MIN_ALLOWED_ERROR_TIMEOUT 500

// ======= Miscelanious: =========

// loop delay info: 
// If it is too large, it might decrease sensitivity and increase lag.
// this can also be used to change smoothness if smooth drive is enabled.
// If it is too little, it might starve the proccessor from energy with too much load.
#define OPCONTROL_LOOP_DELAY 5 // ms
#define AUTON_LOOP_DELAY 5 // ms
#define PI 3.141592653589793
#define ROBOT_WIDTH 3.54331 // in
// accuracy adjustments: the smaller, the more accurate. avoid making it 0
#define ACCURACY 0.01
#define G 9.80665

#endif // VARIABLES_H