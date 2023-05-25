# Building the RADI Pre-base and Base Docker Images

This guide will walk you through the process of building two Docker images, RADI pre-base and RADI base, which are essential building blocks for creating the mini RADI. This steps are not necesary if you just made changes in the RoboticsAcademy repository.

## 1. RADI Pre-base Image
The RADI pre-base Docker image includes all the necessary robotic dependencies for running robotic simulations, as well as the robot models within the CustomRobots directory. If you make any changes in the dependecies or the robot models (RoboticsInfrastructure repository), you will need to build the RADI pre-base image.

### Building the RADI Pre-base Image
The Dockerfile required to build the RADI pre-base image is hosted on the RoboticsInfrastructure GitHub repository. Here are the steps to build the image:

1. **Clone the repository:**
    ```bash
    git clone https://github.com/JdeRobot/RoboticsInfrastructure.git
    ```

2. **Navigate to the Dockerfiles directory:**
    ```bash
    cd RoboticsInfrastructure/scripts
    ```

3. **Build the image:**

    This step is critical as it dictates the generation of the pre-base Docker image that's compatible with either ROS1 or ROS2. This is contingent upon whether changes have been integrated into the main branches of the different versions.

    If you're working from the main branches of ROS1 (`noteic-devel`) or ROS2 (`humble-devel`), simply run the Dockerfile from the corresponding branch. This will automatically generate the appropriate prebase image.

    ```bash
    docker build -t jderobot/robotics-applications:<tag> -f Dockerfile.pre-base .
    ```

    However, if you're working from an auxiliary branch, such as an issue branch, you'll need to substitute the $ROS_DISTRO-devel in the git clone command with the name of your current branch.

    ```dockerfile
    git clone -b <your-branch-name> --depth 1 https://github.com/JdeRobot/RoboticsInfrastructure.git /opt/jderobot
    ```

    Then you can run the dockerfile

    ```bash
    docker build -t jderobot/robotics-applications:<name-of-your-branch> -f Dockerfile.pre-base .
    ```

    Please note that the Docker image tag (jderobot/robotics-applications:name-of-your-branch) should reflect your branch name. This ensures that your Docker image is properly associated with your branch, and prevents any mix-up with Docker images from other branches.

    Remember, this step is crucial for ensuring the successful generation of a Docker pre-base image compatible with the correct version of ROS. Be sure to double-check your branch name before running the Dockerfile.

## 2. RADI base Image

To generate the base Docker image, you simply need to execute the base Dockerfile. This process is generally straightforward and does not require many adjustments. However, there are some situations in which you will need to make modifications.

## Modifying the Robotics Application Manager (RAM)

If you have made changes to the Robotics Application Manager (RAM), which is quite uncommon, you will need to insert the name of your branch in the command that clones the RAM inside the Docker container. You can do this by adding your branch name when running the git clone command in the dockerfile.

```dockerfile
RUN git clone https://github.com/JdeRobot/RoboticsApplicationManager.git -b your-branch-name /RoboticsApplicationManager
```

## Building from a Custom Prebase Image

If you want to generate the base image from a prebase image that you created, you will need to modify the tag inside the Dockerfile so that it builds from the appropriate pre-base. The FROM command at the beginning of your Dockerfile should refer to your custom pre-base Docker image.
    
```dockerfile
FROM jderobot/robotics-applications:<tag>
```
Remember to replace the tag with the tag of your custom pre-base Docker image. This ensures that your Docker base image builds from the correct pre-base image.

After making these modifications, you can proceed with building your base Docker image:

```bash
docker build -t jderobot/robotics-applications:<tag> -f Dockerfile.base .
```


