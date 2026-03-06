# Creating a RADI (RoboticsAcademy Docker Image)

The build.sh script is designed to build a Docker RADI image, from specific branches. It is customizable, allowing the user to choose the version of ROS (Robot Operating System) used, the branches of different repositories, and the tag of the Docker image. Below is a step-by-step guide on how to use the script.

## Purpose

The goal of creating a new non official RADI is to test new dockerfile dependencies or changes done to the robot or worlds models in Robotics Infrastructure.

## Prerequisites

- Docker installed on your machine. You can download Docker from [here](https://www.docker.com/products/docker-desktop).
- Git installed on your machine. You can download Git from [here](https://git-scm.com/downloads).

## Usage

1. **Navigate to the scripts directory**

   ```bash
   cd /scripts/RADI
   ```

2. **Build the Docker image**

   Run the script using the following command:

   ```bash
   ./build.sh -a [ROBOTICS_ACADEMY] -i [ROBOTICS_INFRASTRUCTURE] -m [RAM] -r [ROS_DISTRO] -t [IMAGE_TAG]

   ```

   Each of the parameters is explained below:

`ROBOTICS_ACADEMY`: This is the branch name of the Robotics Academy repository to use. Default value is humble-devel.

`ROBOTICS_INFRASTRUCTURE`: This is the branch name of the Robotics Infrastructure repository to use. Default value is humble-devel.

`RAM`: This is the branch name of the RoboticsApplicationManager repository to use. Default value is humble-devel.

`ROS_DISTRO`: This is the ROS distribution to use. The script currently supports `humble`. Default value is humble.

`IMAGE_TAG`: This is the tag of the Docker image that will be created. Default value is `test`.

3. **Change the tag when launching RA**: launch the tag of the robotics-academy docker image to the one you have just created. If you are using either the developer script or docker compose remember to change the tag in the corresponding **compose*cfg/dev_humble*\*.yaml** file.

## Example

For instance, to build a Docker image using the master branch of the Robotics Academy repository, the humble-devel branch of the Robotics Infrastructure repository, the humble-devel branch of the RAM repository, the humble ROS distribution, and tag the image as my_image, you would run:

```bash
./build.sh -f -a humble-devel -i humble-devel -m humble-devel -r humble -t my_image
```

Use '-f' to force build the base image. If omitted, the base image is created only if it doesn't exist.

### ⚠️ Note for External Contributors (Working from a Fork)

If you are developing from a personal fork of **RoboticsAcademy (RA)** or **RoboticsInfrastructure (RI)**, simply running the `build.sh` script with your branch names (e.g., `-a your-branch -i your-branch`) is **not enough** to apply your changes. 

By default, the Dockerfiles are hardcoded to fetch files and clone repositories directly from the official `JdeRobot` GitHub organization. To build the RADI with your custom fork, you must manually point the Dockerfiles to your repository:

1. Navigate to the `scripts/RADI/` directory and open the relevant Dockerfiles (e.g., `Dockerfile.humble`, `Dockerfile.database`).
2. Search for the official JdeRobot URLs and replace `JdeRobot` with your GitHub username (or your fork's URL). You will typically need to modify:
   * `git clone` commands (e.g., `https://github.com/JdeRobot/...`)
   * `curl` commands fetching raw files (e.g., `https://raw.githubusercontent.com/JdeRobot/...`)
3. Save the changes and run the `build.sh` script again.

*Note: If you are an internal contributor with write access to the official JdeRobot repositories, it is highly recommended to work directly on branches within the official repos instead of forks. This avoids the need to manually modify the Dockerfile URLs.*

## Troubleshooting

If an error occurs while running the script, ensure that:

- All the specified branches exist in their respective repositories.
- The specified ROS distribution is humble.
- You have the necessary permissions to build Docker images on your machine. If not you can run:

```bash
chmod +x build.sh
```

- For more information about the build script:

```bash
./build.sh -h
```

## Searching installed images

If you want to see what images you have installed you can use the command:

```bash
docker images
```

You probably need root permissions to use any command with docker, run them with `sudo`.

## Deleting installed images

If you want to delete images you have installed (be careful not to delete the image you need or you will have to generate it again.) you can use the command:

```bash
docker rmi [image_id]
```

It is possible that an image is still running because it has not finished in some execution, if this happens, you will not be able to use this command. First, list the all the containers (also stopped) with this command:

```bash
docker ps -a
```

Then, remove the containers using the image you want to delete:

```bash
docker rm [docker_id]
```

Finally, you can delete the image.
