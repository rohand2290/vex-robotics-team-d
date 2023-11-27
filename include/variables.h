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
#define LEFT_WHEELS_PORT_2 2 // Defines the 2nd port of the left wheels.
#define LEFT_WHEELS_PORT_3 1 // Defines the 3rd port of the left wheels.
// Right drive:
#define RIGHT_WHEELS_PORT_1 19 // Defines the 1st port of the right wheels.
#define RIGHT_WHEELS_PORT_2 12 // Defines the 2nd port of the right wheels.
#define RIGHT_WHEELS_PORT_3 11 // Defines the 3rd port of the right wheels.

// Intake mechanism:
#define INTAKE_PORT_LEFT 8 // Defines the 1st port of the left side of intake.
#define INTAKE_PORT_RIGHT 13 // Defines the 2nd port of the right side of intake.

// Turret:
#define TURRET_PORT 69 // Defines the port of the turret motor.

// Puncher:
#define PUNCHER_PORT_1 19 // Defines the 1st port of the puncher.
#define PUNCHER_PORT_2 20 // Defines the 2nd port of the puncher.
#define PTO_PORT_1 21
#define PTO_PORT_2 22

// ======== PREFERENCES: =========

// Driving style:
// uncomment the following line if smooth drive is not desired...
#define SMOOTH_CONSTANT 5 // This constant effects the smoothness of the transition, the larger the less smooth.

// Motor speeds:
#define TURRET_SPEED 16 // the turret speed
#define MOTOR_PERCENT 1 // percentage (0-1) for max power to be used (0.75 means the maximum ever reached will be 75%)
#define TURN_PERCENT 1 // percentage (0-1) for max turn sensibility.

// ======= Auton: ================
// trackion wheels:
#define WHEEL_C 2.75 * PI // wheel circumference
// encoder ports:
#define ENCODER_PORT_1 10 // left parralel
#define ENCODER_PORT_2 14 // right parralel
#define ENCODER_PORT_3 3 // center perpendicular
// PID Constants:
#define KP 69
#define KI 69
#define KD 69
#define ERROR_MAX 69
#define ERROR_MIN 69 

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
#define ACCURACY 0.00001

#endif // VARIABLES_H