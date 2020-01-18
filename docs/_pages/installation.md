---
permalink: /installation/
title: "Installation and use"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC installation"
toc_icon: "cog"
---


Robotics-Academy currently supports Linux and Windows operating systems (using docker). Below are the steps for installing the software.

## Ubuntu/Debian

The programming environment is composed of the (a) Gazebo simulator, (b) ROS middleware and (c) the Academy package. All this software is open source so there are alternative ways to install all of them directly from the source code. Currently we use Gazebo-7.4.0, ROS Kinetic and JdeRobot-Academy (2018-06-06) releases.

### Installation

Follow the next steps to have the environment up and running, ready to use.

1. Install ROS framework.

    Add the lastest ROS sources:

    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
    ```bash
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    ```

2. Gazebo sources.

    Add the lastest Gazebo sources:

    ```bash
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable lsb_release -cs main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```

    ```bash
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 67170598AF249743
    ```

3. Install the packages.

    Install the following packages:

    ```bash
    sudo apt-get install ros-melodic-desktop-full
    sudo apt-get install gazebo7
    sudo apt install jderobot-gazebo-assets
    ```

4. Install the JdeRobot-Academy software

    Once you have JdeRobot installed in your system, you can download and install the Academy software. To do so, you must:

    ```bash
    git clone https://github.com/JdeRobot/Academy.git
    ```

### Run Exercises
On the directory of each exercise you will find particular directions to launch the simulated scenario and the academic node where you should write your code. 

Take a look at the [list of exercises](/exercises).


## Windows(x64)

The programming environment is composed of the (a) Docker with Gazebo simulator, (b) JdeRobot middleware for Python and (c) the TeachingRobotics package. All this software is open source so there are alternative ways to install all of them directly from the source code. Currently we use Gazebo-7.4.0, JdeRobot-5.4.1 and TeachingRobotics-0.1.0 releases. JdeRobot Docker includes the Gazebo plugins, models and configuration files to simulate the robot used in the exercises.

### Prerequisites

First check that you have the following dependencies installed. If not, you can install them in the links provided.

- Install checked `env` variables (Is possible that you need restart to run the PATH). You can <a href="https://www.python.org/ftp/python/3.5.2/python-3.5.2-amd64.exe" target="_blank">download here</a>.

- Download qt 5.7 <a href="http://download.qt.io/official_releases/qt/5.7/5.7.0/qt-opensource-windows-x86-msvc2015_64-5.7.0.exe" target="_blank">here</a>.
- Download github Desktop <a href="https://desktop.github.com" target="_blank">from here</a>.
- Download Docker:
  - <a href="https://download.docker.com/win/stable/InstallDocker.msi" target="_blank">Windows 10 x64 proffesional or enterprise</a>
  - <a href="http://www.docker.com/products/docker-toolbox" target="_blank"> Other Windows x64</a>



### Installation

Follow the next four steps to have the environment up and running, ready to use.

1. Upgrade pip.

    Open `CMD` or powershell and upgrade pip typing:

    ```bash
    python -m pip install --upgrade pip
    ```

2. Install depencencies.

    Install the following depencencies

    ```bash
    pip3 install numpy zeroc-ice
    pip3 install pyqt5
    pip3 install opencv-python
    ```

3. Install JdeRobot Python.

    Install JdeRobot Python typing:

    ```bash
    pip3 install http://jderobot.org/store/aitormf/uploads/windows/JdeRobot-0.1.0-py3-none-any.whl
    ```

4. Clone Robotics-Academy repository.

    Download the Academy software. With git Shell clone the repository as in Linux and run the exercices with `CMD` o powershell

    ```bash
    git clone https://github.com/jderobot/academy.git
    ```

_Note: Github repositories are located in `Documents\GitHub`_




### Run Exercises

Open Kinematics of Docker and push "Docker cli".

Run the docker passing the world to use (the first time docker image is downloaded):

```bash
docker run -tiP --rm -p 7681:7681 jderobot/jderobot world [world_name]
```

With `CMD` or PowerShell go to practice directory in Academy and run it:

```bash
python [practice_file][config_file]
```

You can watch a video demonstration.

{% include video id="v00TCr-aSWM" provider="youtube" %}
