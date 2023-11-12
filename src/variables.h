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
#define LEFT_WHEELS_PORT_2 12 // Defines the 2nd port of the left wheels.
#define LEFT_WHEELS_PORT_3 20 // Defines the 3rd port of the left wheels.
// Right drive:
#define RIGHT_WHEELS_PORT_1 1 // Defines the 1st port of the right wheels.
#define RIGHT_WHEELS_PORT_2 4 // Defines the 2nd port of the right wheels.
#define RIGHT_WHEELS_PORT_3 5 // Defines the 3rd port of the right wheels.

// Intake mechanism:
#define INTAKE_PORT_LEFT 10 // Defines the 1st port of the left side of intake.
#define INTAKE_PORT_RIGHT 17 // Defines the 2nd port of the right side of intake.

// Turret:
#define TURRET_PORT 18 // Defines the port of the turret motor.

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
#define INTAKE_IN_SPEED 32 // intake speed inwards
#define INTAKE_OUT_SPEED -64 // intake speed inwards
#define TURRET_SPEED 16 // the turret speed
#define MOTOR_PERCENT 1 // percentage (0-1) for max power to be used (0.75 means the maximum ever reached will be 75%)
#define TURN_PERCENT 1 // percentage (0-1) for max turn sensibility.

// ======= Auton: ================
// trackion wheels:
#define WHEEL_C 2.75 * PI
// encoder ports:
#define ENCODER_PORT_1 69 // left
#define ENCODER_PORT_2 69 // right
#define ENCODER_PORT_3 69 // center


// ======= Miscelanious: =========

// the opcontrol main loop delay. 
// If it is too large, it might decrease sensitivity and increase lag.
// this can also be used to change smoothness if smooth drive is enabled.
// If it is too little, it might starve the proccessor from energy with too much load.
#define OPCONTROL_LOOP_DELAY 5 // ms
#define AUTON_LOOP_DELAY 5 // ms
#define PI (long double)3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811174502841027019385211055596446229489549303819644288109756659334461284756482337867831652712019091456485669234603486104543266482

#endif // VARIABLES_H