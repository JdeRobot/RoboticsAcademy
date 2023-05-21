# Creating a mini RADI (Robotics Application Docker Image)

The build.sh script is designed to build a Docker mini RADI image, from specific branches. It is customizable, allowing the user to choose the version of ROS (Robot Operating System) used, the branches of different repositories, and the tag of the Docker image. Below is a step-by-step guide on how to use the script.

## Prerequisites
- Docker installed on your machine. You can download Docker from [here](https://www.docker.com/products/docker-desktop).
- Git installed on your machine. You can download Git from [here](https://git-scm.com/downloads).

## Usage

1. **Navigate to the scripts directory**
    
    ```bash
    cd /scripts/mini_RADI
    ```

2. **Build the Docker image**
    
    Run the script using the following command:
    
    ```bash
    ./build.sh [ROBOTICS_ACADEMY] [ROBOTICS_INFRASTRUCTURE] [RAM] [ROS_DISTRO] [IMAGE_TAG]

    ```

    Each of the parameters is explained below:

`ROBOTICS_ACADEMY`: This is the branch name of the Robotics Academy repository to use. Default value is master.

`ROBOTICS_INFRASTRUCTURE`: This is the branch name of the Robotics Infrastructure repository to use. Default value is noetic-devel.

`RAM`: This is the branch name of the RoboticsApplicationManager repository to use. Default value is main.

`ROS_DISTRO`: This is the ROS distribution to use. The script currently supports `noetic` and `humble`. Default value is noetic.

`IMAGE_TAG`: This is the tag of the Docker image that will be created. Default value is `test`.
## Example

For instance, to build a Docker image using the master branch of the Robotics Academy repository, the noetic-devel branch of the Robotics Infrastructure repository, the main branch of the RAM repository, the noetic ROS distribution, and tag the image as my_image, you would run:

```bash
./build.sh master noetic-devel main noetic my_image
```

## Troubleshooting

If an error occurs while running the script, ensure that:

- All the specified branches exist in their respective repositories.
- The specified ROS distribution is either noetic or humble.
- You have the necessary permissions to build Docker images on your machine. If not you can run:

```bash
chmod +x build.sh
```