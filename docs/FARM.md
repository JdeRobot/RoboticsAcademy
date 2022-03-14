### [Back to main README.][]

[Back to main README.]: ../README.md

# UniboticsAcademy - Farm

1. [Introduction](#introduction)
2. [Implementation](#implementation)
    1. [Farm's management](#management)
    2. [Conections to Docker API](#dockerapi)
    3. [Conections with users](#conectionsusers)
    4. [GZClient and Console](#vnc)


<a name="introduction"></a>
## Introduction

The farm is a set of computers where the corresponding RADI has been launched and only the users with access enabled
(they used to be URJC students), can connect. In this way, these users don't have to run a local RADI. These computers
are usually used in the following cases as a general rule:

* When the users are connecting through URJC labs' computers. These computers are not able to run Dockers, so the students
must run RADI.

* Underperforming computers. To be able to use Docker and launch containers that run robotic simulations,
it is necessary to have a computer with good performance. If this were not the case, it is possible that Docker cannot be used and that,
even if it could, the RADI would have minimal resources that would prevent a correct execution of the simulations.

<a name="implementation"></a>
## Implementation

<a name="management"></a>
### Farm's management

First of all, it is almost mandatory to have a management panel in Unibotics that allows you to manage the available computers.
For this case, a control panel has been designed where the status of the farm's machines can be seen.

![](figs/granja-ordenadores.png)

Each card represents one of the farm's computers and you can check the status of the computer (it was able to connect to
the computer's Docker API), the state of the container (if there is a RADI running or not) and the version of the RADI
(if is necessary) that is running. As a general rule, the yellow indicator shows that the connection cannot be made, in
the other hand if it is green it shows that the connections work without problems.

Various operations can be performed on each machine such as deleting the farm computer, launching a new RADI,
stop a RADI, launch a RADI that was stopped or launch a new RADI. Also, it is possible to add a new computer
to the farm from the management panel.

Another essential feature of the farm management panel is the ability to stop and delete all
the RADIs and launch a new RADI on each of the computers. It would not be feasible to launch the RADIs one by one in each
computer each time that version is updated.

The view that implements the loading of this management panel is the farm(request) function from the views.py file.

The views that implement each of the functionalities are:

* add_machine(request): A computer is added to the farm.
* delete_machine(request): A farm computer is deleted.
* start_container(request, docker_ip): RADI is launched with a higher version that had previously been stopped on the computer
docker_ip.
* delete_container(request, docker_ip): A RADI running on the docker_ip computer is deleted.
* delete_all(request, docker_ip): All stopped and running RADIs on the docker_ip computer are deleted.
* run_container(request, docker_ip): A new RADI is launched on the docker_ip host.

<a name="dockerapi"></a>
### Conections to Docker API

Connections to farm computers are protected. Specifically, the Docker API can only be accessed from
the farm computers in D3. This is one of the main reasons why updating the functionality of the
farm is complex as changes cannot be committed to D1.

In D3 we have some certificates that are loaded when the server is launched and that allow unibotics.org to be able to
launch requests to the Docker API of the farm machines. These certificates are not available in the repository,
since they are private.

Any request made from the farm management panel goes through the webserver, which in turn connects to
the Docker API of the corresponding computer and sends the command that you want to execute: launch a container, stop a
container etc

<a name="conectionsusers"></a>
### Conections with users (Exercises)

The most complex part of the farm lies in how the users connect with the RADIs that are launched on the computers.
The connection does not take place directly as it happens when RADI is launched locally, if it is not established a
intermediate connection through websockets (ws) with the webserver, which in turn connects with the RADI through another websocket.

These connections are made through Django Consumers and in particular using the module
channels.generic.websocket where the ws that have been created to connect the user with the webserver derive from.
Within each Consumer, another ws is established, which is the one that connects with the different functionalities
of the RADI. The information flow would be as follows: User -> webserver -> RADI -> webserver -> user.

There are 3 Consumers who are in charge of maintaining communication between all the elements.

- CodeConsumer: Consumer through which the code that the user writes in the exercise is sent.
- GuiConsumer: Consumer through which the gui events of the exercise are received.
- LocalConsumer: Consumer that connects to the RADI manager.py to start or stop an exercise.

Both CodeConsumer and GuiConsumer connect to RADI's exercise.py through the ws designated for
it. To understand how it works, it is essential to know how the ws of the exercises work in
[Robotics-Academy](https://github.com/JdeRobot/RoboticsAcademy)

<a name="vnc"></a>
### GZClient y Console

Both GZClient and the Console work through VNC directly, ignoring the described functionality.
previously. Since both information is obtained through a vnc server launched on the farm machines,
In the exercises interface you can directly find IFrames that point to these machines.
To bypass the IP of the machine, a mechanism has been established that makes the content be served directly
from the webserver via the gz_web(request) view.
