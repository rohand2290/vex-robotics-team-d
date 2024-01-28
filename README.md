# vex-robotics-team-d
Code for 6121D team of Conestoga High School.

## File Summaries:
This is meant to explain why the code has been abstracted into seperate files, and not just crammed into **main.cpp**.

### Main.cpp:
Serves as an entry point for all the code. Here is were the main code is kept. This file is abstracted into 6 different functions, with specific goals:

* **on_center_button()** : Called when center button on V5 brain is pressed.
* **initailize()** : Runs the initailization code for settup. excecuted first when program boots up.
* **disabled()** : Called when the robot has been disabled (things like braking make most sense here).
* **competition_initialize()** : This function is called before **autonomous()**, and right after **initialize()**. stuff specific to competition is best here.
* **autonomous()** : This function is called when our bot is on autonomous mode. All driver code for autonomous mode goes in here.
* **opcontrol()** : This function is called during driver mode, where the driver has to controll it via a Controller.

### Variables.h:
This file serves as a clean mediator between our code, and our data. This file includes universal constants that can be fine tuned. 
It can store a range of things including ports, numerical constants, etc. 

### Depend.h:
This file serves as a universal include function. If items need to be imported to another file, include just this file will include everything.

### Items.cpp:
This file contains the definitions for all the functions of the Items class.

### Items.h:
This file contains the Items class. The Items class stores all the physical hardware objects used throughout the program. This class ensures that the class objects are deleted safely, and that the program doesnt crash at the end.

### Robot.cpp:
This file contains the definition for all the functions of the Robot class.

### Robot.h:
This file contains the Robot Class. This class is an OOP representation of our robot, in order to organize our code well. This class contains all robot specific functions, like the drive train.

### Waypoint.cpp:
Contains all definitions for the functions in the Waypoint class.

### Waypoint.h:
This file contains the Waypoint class. This class represents a waypoint to be followed by the bot in autonomous as part of the PID Algorithm.

Please keep in mind that there is no hesitation in abstracting further (a file for motor stuff, math stuff, sensor stuff, etc.), as it might be crucial in the long run.
Just make sure to include it in this README file with a good explaination, so we can avoid confusion.
## Auton Commands:

* move: takes 1 parameter (first param) as inches. Moves bot x inches. negative inches for backwards, positive for forwards.
* turn: takes 1 parameter (first param) as degrees. Turn bot that many degrees relative to current position. Positive for clockwise, negative for counterclockwise.
* in: activates intake...
* out: activates outake...
* wings: toggles state of front wings...
* bwings: toggles state of back wings...
* stop: stops the intake.
* rise: engages PTO
* fall: disengages PTO
* coast: sets drive train brake mode to COAST
* hold: sets drive train brake mode to HOLD
* wait: takes 1 parameter (first param) as milliseconds. halts the program for that many milliseconds.
* spamcata: runs the cata untill program is terminated.
* power: takes 2 parameters. First is analog range [-127, 127], second is milliseconds. Power will manually set drive motors to the given first parameter for the given milliseconds given in second parameter.
* cata: takes 1 parameter (first param) in whole numbers. Runs cata for the given amount of times
* precise: Sets error mode to precise, making movements precise, steady, and slow after this command.
* pass: Sets the error mode to pass, making movements fast, less steady, and little bit inacurate (depending on constants)

