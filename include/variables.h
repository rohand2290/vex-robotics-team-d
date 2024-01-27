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
#define LEFT_WHEELS_PORT_1 2 // Defines the 1st port of the left wheels.
#define LEFT_WHEELS_PORT_2 3 // Defines the 2nd port of the left wheels.
#define LEFT_WHEELS_PORT_3 4 // Defines the 3rd port of the left wheels.
// Right drive:
#define RIGHT_WHEELS_PORT_1 7 // Defines the 1st port of the right wheels.
#define RIGHT_WHEELS_PORT_2 8 // Defines the 2nd port of the right wheels.
#define RIGHT_WHEELS_PORT_3 9 // Defines the 3rd port of the right wheels.

// Intake mechanism:
#define INTAKE_PORT_LEFT 19 // Defines the 1st port of the left side of intake.
#define INTAKE_PORT_RIGHT 69 // Defines the 2nd port of the right side of intake.

// Catapult:
#define CATA_PORT 10 // Defines the port of the cata.
#define CATA_ENCODER 1 // Define the encoder port for cata
#define CATA_REST 90 // Defines the resting position of the cata...

// Pistons:
#define INTAKE_PISTON_PORT 69
#define PTO 'C'
#define WING_1_PORT 'B'
#define WING_2_PORT 'A'

// IMU:
#define IMU_PORT 6

// ======== PREFERENCES: =========

// Motor speeds:
#define FLYWHEEL_SPEED 255 // the turret speed

// ============== Auton: ================
// gamemode:
//#define SKILLS // uncomment if not using skills
#define JOYSTICK_DEADZONE 10 // deadzone analog value for which drive wont be triggered.
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
// max values ever for PID:
#define MAX_I_VAL 360
#define MIN_I_VAL -360
// power:
#define POWER_KP 0.899
#define POWER_KI 0.00001
#define POWER_KD 0
// turn:
#define TURN_KP 0.17
#define TURN_KI 0.00001
#define TURN_KD 0.001
// cata:
#define CATA_KP 1
// break:
#define BREAK_KP 0.1
// swing:
#define SWING_KP 1
#define SWING_KI 0
#define SWING_KD 0
// ======================= ODOM: ========================
#define ERROR_MEASUREMENT 0.05
#define MECH_ADVANTAGE 0.6 // mechanical advantage between drive motors and wheels.

// maximum allowed error:
#define MIN_ALLOWED_ERROR 1.7
#define MIN_ALLOWED_ERROR_DEG 1
#define MIN_ALLOWED_ERROR_TIME 100
#define MIN_ALLOWED_ERROR_TIMEOUT 2500

// ======= Miscelanious: =========

// loop delay info: 
// If it is too large, it might decrease sensitivity and increase lag.
// this can also be used to change smoothness if smooth drive is enabled.
// If it is too little, it might starve the proccessor from energy with too much load.
#define OPCONTROL_LOOP_DELAY 5 // ms
#define AUTON_LOOP_DELAY 1 // ms
#define PI 3.141592653589793
#define ROBOT_WIDTH 3.54331 // in
// accuracy adjustments: the smaller, the more accurate. avoid making it 0
#define ACCURACY 0.01
#define G 9.80665

#endif // VARIABLES_H