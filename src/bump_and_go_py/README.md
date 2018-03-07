                        BUMP AND GO EXCERSISE
                        =====================

The intention of this excersise is to program a basic behaviour of bump-spin using
a finite state machine. For that, we will use JdeRobot visualStates tool, that 
allows you to create your own states machine in an intuitive way.

////////////////////////////////////////////////////////////////////////////////
                           E X E C U T I O N 
////////////////////////////////////////////////////////////////////////////////

Once created the state machine and set the code for each state,
To launch the example you only have to follow the following steps:

1. Run Gazebo:
     *Execution without seeing the world: 
`roslaunch kobuki-simple-ros.launch`
     *Execution watching the world: 
`roslaunch kobuki-simple-ros-gui.launch`
2. Execution of the bum&go component: 
`./bump_and_go.py bump_and_go.yml --displaygui=true`
* The code of the machine will start its execution automatically.


* To simplify the closure of the environment, just close the window (s). 
  Ctrl + C will give problems.

////////////////////////////////////////////////////////////////////////////////

## How to do the practice
We will use the visualStates tool to carry out this practice.
To do this, you have to perform several tasks:

1. Open the visualStates component:
`$ cd visualStates_py`
`$./visualStates.py`
You will see a graphical interface in whose toolbar you have several options,
such as: FIle, Figures, Data, Actions, Help. At first, the work area is empty.

2. Create the state machine:
Click on `Figures -> State`. Then click on the part of the work area work in 
which you want to put the state 1 (then you can move it if you need it).
Add as many states as you need. Making a left click on each state, you are able 
to change its name `-> Rename`. Set the state you want as the main one,
by left clicking on the state and pressing `-> Make Initial`.
You will see that each change you make will be recorded in the scheme on the left 
of the interface. It's time to add the transitions for each state. To do this, 
we access the tab Figures again as `Figures -> Transition`. Clicking on the origin 
state and then making another click on the destination state you will establish 
the transition between the source and destination state. You can change
the name of the transitions in the same way as for the states.

3. Establish the robot configuration file. To do this, access
to the tab `Actions -> Config File` and set the following:

For simulated turtlebot:
        Server Type      Name                       Topic                          Interface
    -      ROS         myMotors      /turtlebotROS/mobile_base/commands/velocity    Motors
    -      ROS         myLaser       /turtlebotROS/laser/scan                       Laser
    -      ROS         myPose        /turtlebotROS/odom                             Pose3d

For real turtlebot:
        Server Type      Name                       Topic            Interface
    -      ROS         myMotors      /mobile_base/commands/velocity    Motors
    -      ROS         myLaser       /scan                             Laser
    -      ROS         myPose        /odom                             Pose3d

* In the "Name" cell you will put the name with which you will reference the 
sensors or actuators of the robot. Use these names to send orders to the robot 
or receive data from it in the code that you establish in the next step.

4. Now that you have your finite state machine, you must set the code
that manages the status changes:

First, access `Data -> Variables` and write there the global variables that your 
program needs (make sure that the `Python` tab is checked).
In the same way, access `Data -> Functions` and write the procedures you need.
Now, in each state and transition, left click and access `-> Code`, where
you must establish the code that is executed in each state or transition. For 
transitions, make sure that in the "Transition Type" section is checked the 
option `Conditional`. Then, in the "Condition" area, add the code of the
condition that determines the passage from one state to another.

* Remember to save all the changes `File -> Save As`. Choose the name "bump_and_go"
for your automaton (or use the name that you have put in this step when you try
to execute) and save it in the folder of this practice.

5. Once all this is done, you only will have to save and go to `Actions -> Generate Python`
to generate in your working folder the files bump_and_go.py, bump_and_go.yml and
bump_and_go.xml Then you can execute the result as specified above.

## SUGESTIONS
This practice is easy to carry out if you use 3 states:
    - Go Straight
    - Go Back
    - Spin
Each one with its own transition. Nonetheless, there are so many ways to do it.

### API
* IF YOU HAVE USED THE SAME NAMES THAT IN THIS FILE YOU CAN USE THE API AS IT
APPEARS UNDER. IF YOU HAVE CHANGED THEM, RESPECT THE NAMES YOU HAVE USED.
`USE THE OBJECT self.interfaces TO REFERENCE VARIABLES AND FUNCTIONS YOU CREATED IF YOU ARE`
`ESTABLISHING THE CODE OF STATES AND TRANSITIONS`
* self.interfaces.myMotors.sendV(vel) - to set the linear velocity
* self.interfaces.myMotors.sendW(vel) - to set the angular velocity
* self.interfaces.myLaser.getLaserData() - to obtain the laser sensor data
* self.interfaces.myFunction() - to execute myFunction().
* self.interfaces.myVariable - to use the global variable myVariable (in this case, Bool type)
`IF YOU ARE ESTABLISHING GLOBAL VARIABLES OR FUNCTIONS, IT IS SUFFICIENT WITH "self.variable" or`
`"def myFunction(self)"`

### Own API
The implementation of a finite and deterministic state machine is offered. The student will be able to define
his/her own automatons in the way that suits him/her best.


## Demonstrative video
https://youtu.be/o-SAe_qwOMc 

##  Launch Real turtlebot
* Install following packages:
`sudo apt install ros-kinetic-rplidar-ros ros-kinetic-kobuki-node ros-kinetic-hokuyo-node ros-kinetic-laser-filters`

* f not already in the dialout group: 
`sudo usermod -a -G dialout $USER`
* first, connect laser (Hokuyo has 2 wires), then turn on turtlebot and plug it.

* If your Turtlebot has a Hokuyo laser use:
`roslaunch turtlebot-hokuyo.launch`

* If your Turtlebot has a rplidar laser use:
`roslaunch turtlebot-rplidar.launch`
