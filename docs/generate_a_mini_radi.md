# Creating a mini RADI (Robotics Application Docker Image)

This guide provides instructions on how to generate a mini RADI (Robotics Application Docker Image) for the Robotics Academy project. The following steps will guide you to create a Docker image that will contain all the necessary dependencies, the RAM (Robotics Application Manager), and the content from the Robotics Academy repository.

## Prerequisites
- Docker installed on your machine. You can download Docker from [here](https://www.docker.com/products/docker-desktop).
- Git installed on your machine. You can download Git from [here](https://git-scm.com/downloads).

## Steps to Create RADI

1. **Navigate to the scripts directory**
    
    ```bash
    cd /path/to/scripts
    ```

2. **Build the Docker image**
    - Execute the Docker build command with the specified tag. Replace `<tag>` with your desired version number or tag.
    
    ```bash
    docker build -t "jderobot/robotics-academy:<tag>" .
    ```

## Dockerfile Configuration

The Dockerfile used for building the image is configured to use a base image that contains all the required robotic dependencies and the RAM (Robotics Application Manager). It is also set to clone the RoboticsAcademy repository from the `master` branch.

To build a RADI from a different branch, you will need to modify the Dockerfile to point to the desired branch. For example, if you want to clone from the issue-100 branch, the corresponding line in the Dockerfile would look like:
```dockerfile
RUN git clone --depth 10 https://github.com/JdeRobot/RoboticsAcademy.git -bissue-100
```