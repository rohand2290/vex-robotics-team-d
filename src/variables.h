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
#define INTAKE_IN_SPEED 32 // intake speed inwards
#define INTAKE_OUT_SPEED -64 // intake speed inwards
#define TURRET_SPEED 16 // the turret speed
#define MOTOR_PERCENT 1 // percentage (0-1) for max power to be used (0.75 means the maximum ever reached will be 75%)
#define TURN_PERCENT 1 // percentage (0-1) for max turn sensibility.

// ======= Auton: ================
// trackion wheels:
#define WHEEL_C 3.54331 * PI // wheel circumference
// encoder ports:
#define ENCODER_PORT_1 10 // left parralel
#define ENCODER_PORT_2 20 // right parralel
#define ENCODER_PORT_3 3 // center perpendicular
// accuracy adjustments: the smaller, the more accurate. avoid making it 0
#define ODOM_ACCURACY 0.00001
#define CHECK_FOR_ENV_FORCES false // check for forces from outside the environment.

// ======= Miscelanious: =========

// loop delay info: 
// If it is too large, it might decrease sensitivity and increase lag.
// this can also be used to change smoothness if smooth drive is enabled.
// If it is too little, it might starve the proccessor from energy with too much load.
#define OPCONTROL_LOOP_DELAY 5 // ms
#define AUTON_LOOP_DELAY 5 // ms
#define PI (long double)3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811174502841027019385211055596446229489549303819644288109756659334461284756482337867831652712019091456485669234603486104543266482
#define ROBOT_WIDTH 3.54331 // in

#endif // VARIABLES_H