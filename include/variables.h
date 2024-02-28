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
#define LEFT_WHEELS_PORT_1 11 // Defines the 1st port of the left wheels.
#define LEFT_WHEELS_PORT_2 2 // Defines the 2nd port of the left wheels.
#define LEFT_WHEELS_PORT_3 1 // Defines the 3rd port of the left wheels.
// Right drive:
#define RIGHT_WHEELS_PORT_1 20 // Defines the 1st port of the right wheels.
#define RIGHT_WHEELS_PORT_2 9 // Defines the 2nd port of the right wheels.
#define RIGHT_WHEELS_PORT_3 10 // Defines the 3rd port of the right wheels.

// Intake mechanism:
#define INTAKE_PORT_LEFT 8 // Defines the 1st port of the left side of intake.
#define INTAKE_PORT_RIGHT 4 // Defines the 2nd port of the right side of intake.

// Catapult:
#define CATA_PORT 69 // Defines the port of the cata.
#define CATA_ENCODER 3 // Define the encoder port for cata
#define CATA_REST 80 // Defines the resting position of the cata...

// Pistons:
#define INTAKE_PISTON_PORT 69
#define PTO_CATA 'H'
#define PTO_INTAKE 'C'
#define PTO_CLIMB 'H'
#define WING_1_PORT 'F'
#define WING_2_PORT 'G'

// IMU:
#define IMU_PORT 19

// ======== PREFERENCES: =========

// Motor speeds:
#define FLYWHEEL_SPEED 255 // the turret speed

// ============== Auton: ================
// gamemode:
//#define SKILLS
#define JOYSTICK_DEADZONE 5 // deadzone analog value for which drive wont be triggered.
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
#define POWER_KP 9.5 //0.899
#define POWER_KI 0.001
#define POWER_KD 0.1
// turn:
#define TURN_KP 1.225
#define TURN_KI 0.0000001
#define TURN_KD 0.01
// cata:
#define CATA_KP 2.5
// break:
#define BREAK_KP 0.1
// swing:
#define SWING_KP 1
#define SWING_KI 0
#define SWING_KD 0
// raw movement cosine constants:
#define COS_FACTOR 0.95 // set to 1 if doesnt work
#define SENSITIVE_FACTOR 2.7 // leave same if doesnt work
// ======================= ODOM: ========================
#define ERROR_MEASUREMENT 0.05
#define MECH_ADVANTAGE 0.70356244544 // mechanical advantage between drive motors and wheels.

// maximum allowed error:
#define MIN_ALLOWED_ERROR 1.7
#define MIN_ALLOWED_ERROR_DEG 4.5
#define MIN_ALLOWED_ERROR_TIME 100
#define MIN_ALLOWED_ERROR_TIMEOUT_DEG 1000
#define MIN_ALLOWED_ERROR_TIMEOUT 3000

// ======= Miscelanious: =========

// loop delay info: 
// If it is too large, it might decrease sensitivity and increase lag.
// this can also be used to change smoothness if smooth drive is enabled.
// If it is too little, it might starve the proccessor from energy with too much load.
#define OPCONTROL_LOOP_DELAY 5 // ms
#define AUTON_LOOP_DELAY 1 // ms
#define PI 3.141592653589793
#define ROBOT_WIDTH 3.54331 // in
#define SLEW_RATE 270 // some unit...
// accuracy adjustments: the smaller, the more accurate. avoid making it 0
#define ACCURACY 0.25
#define G 9.80665

#endif // VARIABLES_H