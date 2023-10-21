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

Please keep in mind that there is no hesitation in abstracting further (a file for motor stuff, math stuff, sensor stuff, etc.), as it might be crucial in the long run.
Just make sure to include it in this README file with a good explaination, so we can avoid confusion.
