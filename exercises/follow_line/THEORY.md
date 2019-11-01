## Theory
PID Control is the main fundamental behind this exercise. To understand PID Control, let us first understand what is Control in general.

### Control System
A system of devices or set of devices, that manages, commands, directs or regulates the behavior of other devices or systems to achieve the desired results. Simply speaking, a system which controls other systems. Control Systems help a robot to execute a set of commands precisely, in the presence of unforseen errors.

![Control System](./assets/ControlSystems.jpg)

### Types of Control System
#### Open Loop Control System
A control system in which the control action is completley independent of the output of the system. A manual control system is on Open Loop System.

#### Closed Loop Control System
A control system in which the ouput has an effect on the input quantity in such a manner that the input will adjust itself based on the output generated. An open loop system can be converted to a closed one by providing a feedback.

![Types of Control Systems](./assets/TypesofControlSystems.jpg)

### PID Control
A control loop mechanism employing feedback. A PID Controller continously calculates an error value as the difference between desired output and the current output and applies a correction based on proporional, integral and derivative terms(denoted by P, I, D respectively).

#### Proportional
Proportional Controller gives an output which is proportional to the current error. The error is multiplied with a proportionality constant to get the output. And hence, is 0 if the error is 0.
#### Integral
Integral Controller provides a necessary action to eliminate the offset error which is accumalted by the P Controller.It integrates the error over a period of time until the error value reaches to zero.
#### Derivative
Derivative Controller gives an output depending upon the rate of change or error with respect to time. It gives the kick start for the output thereby increasing system response.

![PID Control Equation](./assets/PID.png)

#### Tuning Methods
In order for the PID equation to work, we need to determine the constants of the equation. There are 3 constants called the gains of the equation. We have 2 main tuning methods for this.

**Trial and Error**: It is a simple method of PID controller tuning. While system or controller is working, we can tune the controller. In this method, first we have to set Ki and Kd values to zero and increase proportional term (Kp) until system reaches to oscillating behavior. Once it is oscillating, adjust Ki (Integral term) so that oscillations stops and finally adjust D to get fast response.

**Zeigler Nichols method**: Zeigler-Nichols proposed closed loop methods for tuning the PID controller. Those are continuous cycling method and damped oscillation method. Procedures for both methods are same but oscillation behavior is different. In this, first we have to set the p-controller constant, Kp to a particular value while Ki and Kd values are zero. Proportional gain is increased till system oscillates at constant amplitude.

## References
For a more practical explanation, have a look at the following links:

[Link1](https://accautomation.ca/tag/pid-control-car-analogy/)

[Link2](https://www.youtube.com/watch?v=UR0hOmjaHp0)

Some other references:

https://www.electrical4u.com/control-system-closed-loop-open-loop-control-system/

https://en.wikipedia.org/wiki/PID_controller

https://www.elprocus.com/the-working-of-a-pid-controller/

https://www.tutorialspoint.com/control_systems/control_systems_introduction.htm

https://instrumentationtools.com/open-loop-and-closed-loop-animation/

https://trinirobotics.com/2019/03/26/arduino-uno-robotics-part-2-pid-control/

