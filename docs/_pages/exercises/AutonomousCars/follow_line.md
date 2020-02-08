---
permalink: /exercises/follow_line/
title: "Visual Follow Line"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Visual Follow Line"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/follow_line/formula1_circuit.png
    image_path: /assets/images/exercises/follow_line/formula1_circuit.png
    alt: "Racing circuit."
    title: "Racing circuit."
  - url: /assets/images/exercises/follow_line/formula1.png
    image_path: /assets/images/exercises/follow_line/formula1.png
    alt: "First Person."
    title: "First Person."
  - url: /assets/images/exercises/follow_line/formula1_2.png
    image_path: /assets/images/exercises/follow_line/formula1_2.png
    alt: "Model."
    title: "Model."
  
gifs:
  - url: /assets/images/exercises/follow_line/oscillations.gif
    image_path: /assets/images/exercises/follow_line/oscillations.gif
    alt: "examples"
    title: "examples"
  - url: /assets/images/exercises/follow_line/slowresponse.gif
    image_path: /assets/images/exercises/follow_line/slowresponse.gif
    alt: "examples"
    title: "examples"
  


youtubeId: eNuSQN9egpA

---

## Goal

The goal of this exercise is to perform a PID reactive control capable of following the line painted on the racing circuit.

{% include gallery caption="Gallery" %}

The students program a Formula1 car in a race circuit to follow the red line in the middle of the road.

## Installation 

To launch the infrastructure of this practice, first set up the gazebo sources, then launch the simulator with the appropriate scenario:

```bash
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```

or add them directly to your bashrc to run automatically whenever you open a terminal:

```bash
echo 'source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh' > ~/.bashrc
```

```bash
echo 'source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh' > ~/.bashrc
```

```bash
source ~/.bashrc
```

## How to perform the exercise?
To carry out the exercise, you have to edit the file `MyAlgorithms.py` and insert in it your code, which gives intelligence to the autonomous car.

### Where to insert the code?
In the `MyAlgorithm.py` file,

```python
def execute(self):
    #GETTING THE IMAGES
    image = self.getImage()

    # Add your code here
    print "Runing"

    #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
    #self.motors.sendV(10)
    #self.motors.sendW(5)

    #SHOW THE FILTERED IMAGE ON THE GUI
    self.set_threshold_image(image)
```

### Application Programming Interface

* `self.getImage()` - to get the image
* `self.motors.sendV()` - to set the linear speed
* `self.motors.sendW()` - to set the angular velocity
* `self.set_threshold_image()` - allows you to view a debug image or with relevant information. It must be an image in RGB format (Tip: np.dstack())

## How to run your solution?

Navigate to the follow_line directory

```bash
cd exercises/follow_line
```

Launch Gazebo with the f1_simple_circuit world through the command 

```bash
roslaunch ./launch/simple_line_follower_ros.launch
```

Then you have to execute the academic application, which will incorporate your code:

```bash
python2 ./follow_line.py follow_line_conf.yml
```

## Theory

PID Control is the main fundamental behind this exercise. To understand PID Control, let us first understand what is Control in general.

### Control System

A system of devices or set of devices, that manages, commands, directs or regulates the behavior of other devices or systems to achieve the desired results. Simply speaking, a system which controls other systems. Control Systems help a robot to execute a set of commands precisely, in the presence of unforseen errors.

![Control System]({{ site.url }}/RoboticsAcademy/assets/images/exercises/follow_line/ControlSystems.jpg)

### Types of Control System
#### Open Loop Control System
A control system in which the control action is completley independent of the output of the system. A manual control system is on Open Loop System.

#### Closed Loop Control System
A control system in which the ouput has an effect on the input quantity in such a manner that the input will adjust itself based on the output generated. An open loop system can be converted to a closed one by providing a feedback.

![Types of Control Systems]({{ site.url }}/RoboticsAcademy/assets/images/exercises/follow_line/TypesofControlSystems.jpg)

### PID Control
A control loop mechanism employing feedback. A PID Controller continously calculates an error value as the difference between desired output and the current output and applies a correction based on proporional, integral and derivative terms(denoted by P, I, D respectively).

- **Proportional**

Proportional Controller gives an output which is proportional to the current error. The error is multiplied with a proportionality constant to get the output. And hence, is 0 if the error is 0.

- **Integral**

Integral Controller provides a necessary action to eliminate the offset error which is accumalted by the P Controller.It integrates the error over a period of time until the error value reaches to zero.

- **Derivative**

Derivative Controller gives an output depending upon the rate of change or error with respect to time. It gives the kick start for the output thereby increasing system response.

![PID Control Equation]({{ site.url }}/RoboticsAcademy/assets/images/exercises/follow_line/PID.png)

- **Tuning Methods**

In order for the PID equation to work, we need to determine the constants of the equation. There are 3 constants called the gains of the equation. We have 2 main tuning methods for this.

- **Trial and Error**

It is a simple method of PID controller tuning. While system or controller is working, we can tune the controller. In this method, first we have to set Ki and Kd values to zero and increase proportional term (Kp) until system reaches to oscillating behavior. Once it is oscillating, adjust Ki (Integral term) so that oscillations stops and finally adjust D to get fast response.

- **Zeigler Nichols method**

Zeigler-Nichols proposed closed loop methods for tuning the PID controller. Those are continuous cycling method and damped oscillation method. Procedures for both methods are same but oscillation behavior is different. In this, first we have to set the p-controller constant, Kp to a particular value while Ki and Kd values are zero. Proportional gain is increased till system oscillates at constant amplitude.

### Real Life Example

[1](https://accautomation.ca/tag/pid-control-car-analogy/)
[2](https://www.youtube.com/watch?v=UR0hOmjaHp0)

## Hints
Simple hints provided to help you solve the follow_line exercise.

### Detecting the Line to Follow
The first task of the assignment is to detect the line to be followed. This can be achieved easily by **filtering the color of the line** from the image and applying basic image processing to find the point or line to follow, or in Control terms our *Set Point*. Refer to these links for more information:

[1](https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/)
[2](https://stackoverflow.com/questions/10469235/opencv-apply-mask-to-a-color-image)
[3](https://stackoverflow.com/questions/22470902/understanding-moments-function-in-opencv)

### Coding the Controller
The Controller can be designed in various configurations. 3 configurations have been descirbed in detail below:

- **P Controller**
The simplest way to do the assignment is using the P Controller. Just find the error which is the difference between our *Set Point* (The point where our car should be heading) and the *Current Output* (Where the car is actually heading). Keep adjusting the value of the constant, till we get a value where there occurs no [**unstable oscillations**](#Illustrations) and no [**slow response**](#Illustrations).

- **PD Controller**
This is an intersting way to see the effect of Derivative on the Control. For this, we need to calculate the derivative of the output we are receiving. Since, we are dealing with *discrete outputs in our case, we simply calculate the difference between our previous error and the present error*, then adjust the proportional constant. Adjust this value along with the P gain to get a good result.

- **PID Controller**
This is the complete implemented controller. Now, to add the I Controller we need to integrate the output from the point where error was zero, to the present output. While dealing with discrete outputs, we can acheive this using *accumalated error*. Then, comes the task of adjustment of gain constants till we get our desired result.

### Illustrations

{% include gallery id="gifs" caption="Unstable Oscillations (left) - Slow Response (right)" %}


## Demonstrative Video

{% include youtubePlayer.html id=page.youtubeId %}

## Contributors

- Contributors: [Alberto Martín](https://github.com/almartinflorido), [Francisco Rivas](https://github.com/chanfr), [Francisco Pérez](https://github.com/fqez), [Jose María Cañas](https://github.com/jmplaza), [Nacho Arranz](https://github.com/igarag).
- Maintaied by [Pankhuri Vanjani](https://github.com/pankhurivanjani) and [Sakshay Mahna](https://github.com/SakshayMahna).

## References

[1](https://www.electrical4u.com/control-system-closed-loop-open-loop-control-system/)
[2](https://en.wikipedia.org/wiki/PID_controller)
[3](https://www.elprocus.com/the-working-of-a-pid-controller/)
[4](https://www.tutorialspoint.com/control_systems/control_systems_introduction.htm)
[5](https://instrumentationtools.com/open-loop-and-closed-animation-loop/)
[6](https://trinirobotics.com/2019/03/26/arduino-uno-robotics-part-2-pid-control/)
[7](http://homepages.math.uic.edu/~kauffman/DCalc.pdf)