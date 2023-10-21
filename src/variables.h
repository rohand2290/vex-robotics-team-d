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
#define LEFT_WHEELS_PORT_1 10 // Defines the 1st port of the left wheels.
#define LEFT_WHEELS_PORT_2 11 // Defines the 2nd port of the left wheels.
#define LEFT_WHEELS_PORT_3 12 // Defines the 3rd port of the left wheels.
// Right drive:
#define RIGHT_WHEELS_PORT_1 13 // Defines the 1st port of the right wheels.
#define RIGHT_WHEELS_PORT_2 14 // Defines the 2nd port of the right wheels.
#define RIGHT_WHEELS_PORT_3 15 // Defines the 3rd port of the right wheels.

// Intake mechanism:
#define INTAKE_PORT_LEFT 16 // Defines the 1st port of the left side of intake.
#define INTAKE_PORT_RIGHT 17 // Defines the 2nd port of the right side of intake.

// Turret:
#define TURRET_PORT 18 // Defines the port of the turret motor.

// Puncher:
#define PUNCHER_PORT_1 19 // Defines the 1st port of the puncher.
#define PUNCHER_PORT_2 20  

// ======== PREFERENCES: =========

// Driving style:
// #define TANK // comment if a tank style drive is not desired...

// Motor speeds:
#define INTAKE_IN_SPEED 32 // intake speed inwards
#define INTAKE_OUT_SPEED -64 // intake speed inwards
#define TURRET_SPEED 16 // the turret speed
#define MOTOR_PERCENT 1 // percentage (0-1) for max power to be used (0.75 means the maximum ever reached will be 75%)

// ======= Miscelanious: =========

// the opcontrol main loop delay. 
// If it is too large, it might decrease sensitivity and increase lag.
// If it is too little, it might starve the proccessor from energy with too much load.
#define OPCONTROL_LOOP_DELAY 40 // ms

#endif // VARIABLES_H