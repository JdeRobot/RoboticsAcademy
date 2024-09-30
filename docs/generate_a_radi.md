# Creating a RADI (RoboticsAcademy Docker Image)

The build.sh script is designed to build a Docker RADI image, from specific branches. It is customizable, allowing the user to choose the version of ROS (Robot Operating System) used, the branches of different repositories, and the tag of the Docker image. Below is a step-by-step guide on how to use the script.

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

`ROBOTICS_ACADEMY`: This is the branch name of the Robotics Academy repository to use. Default value is master.

`ROBOTICS_INFRASTRUCTURE`: This is the branch name of the Robotics Infrastructure repository to use. Default value is noetic-devel.

`RAM`: This is the branch name of the RoboticsApplicationManager repository to use. Default value is main.

`ROS_DISTRO`: This is the ROS distribution to use. The script currently supports `noetic` and `humble`. Default value is noetic.

`IMAGE_TAG`: This is the tag of the Docker image that will be created. Default value is `test`.
## Example

For instance, to build a Docker image using the master branch of the Robotics Academy repository, the noetic-devel branch of the Robotics Infrastructure repository, the main branch of the RAM repository, the noetic ROS distribution, and tag the image as my_image, you would run:

```bash
./build.sh -f -a master -i noetic-devel -m main -r noetic -t my_image
```
Use '-f' to force build the base image. If omitted, the base image is created only if it doesn't exist.
## Troubleshooting

If an error occurs while running the script, ensure that:

- All the specified branches exist in their respective repositories.
- The specified ROS distribution is either noetic or humble.
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