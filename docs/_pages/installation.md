---
permalink: /installation/
title: "INSTALLATION INSTRUCTIONS"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC installation"
toc_icon: "cog"
---


Robotics-Academy currently supports **Linux** operating system. These installation instructions have been prepared for Ubuntu Linux 18.04 (LTS). Don't install everything listed here, just follow the directions on the Academy exercise you want to perform. For instance, not all the specific infrastructure packages are required for every exercise, and they can be large packages.

1. *Generic infrastucture*
    - Required
    - ROS middleware, Gazebo simulator, JdeRobot-base, JdeRobot-assets

2. *Specific infrastructure*
    - Optional, depends on the exercise
    - Some exercises require also additional software pieces such as OpenCV, MoveIt!, MavROS, PX4, VisualStates, OpenMotionPlanningLibrary, etc.

3. *JdeRobot Academy source code*
    - Required
    - Academy includes templates for the exercises

# Generic infrastructure

## ROS Middleware

Supported release is ROS-Melodic. It can be easily installed from official Debian packages, maintained by OpenRobotics organization.

1. Add the lastest ROS source:

    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```

    ```bash
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    ```

2. Install the official ROS Melodic Debian package

    ```bash
    sudo apt-get install ros-melodic-desktop-full
    ```

## Gazebo simulator

Supported release is Gazebo-9. It can be easily installed from official Debian packages, maintained by OpenRobotics organization. You can safely ignore this step as Gazebo-9 will be automatically installed with ROS Melodic package.

1. Add the lastest Gazebo source:

    ```bash
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable lsb_release -cs main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```

    ```bash
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 67170598AF249743
    ```

2. Install the official Gazebo package:

    ```bash
    sudo apt-get install gazebo9
    ```

## JdeRobot/base

Supported release is 6.0.0.
It contains ROS drivers not included in the official ROS packages.

1. Add the latest JdeRobot-base source

    ```bash
    sudo sh -c 'cat<<EOF>/etc/apt/sources.list.d/jderobot.list

    deb [arch=amd64] http://wiki.jderobot.org/apt bionic main
    EOF'
    ```
   
2. Get and add the public key from the JdeRobot repository

    ```bash
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 24E521A4
    ```

3. Update the repositories

    ```bash
    sudo apt update
    ```

4. Install JdeRobot

    ```bash
    sudo apt install jderobot
    ```


## JdeRobot/assets

Supported release is 6.0.0
It contains Gazebo world files and configuration files required for the Academy exercises.

1. Install JdeRobot-assets
    ```bash
    sudo apt install jderobot-gazebo-assets
    ```


# Specific infrastructure

## OpenCV
## MavROS
## PX4
## MoveIt!


# Academy source code

Once you have generic and specific infrastructure installed in your system, you can download and install the JdeRobot Academy software.

1. Clone the Academy software repository

    ```bash
    git clone https://github.com/JdeRobot/Academy.git
    ```

<!---

# Installation on Windows(x64)  DEPRECATED

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


--->