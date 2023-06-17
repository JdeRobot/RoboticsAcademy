# New RAM development documentation

## Building the new RAM RADI

Now build.sh script supports some parameters and help.

```commandline
scripts/> ./build.sh -h
Generates RoboticsAcademy RADI image

Syntax: build.sh [-h] [-f] <tag>
options:
-h   Print this Help.
-f   Force creation of base image. If ommited, the base image is created only if 
     it doestn exist.
```           

The DockerFile in the branch includes a commented out entrypoint by the end of the file. This has been done to
ease development and debugging of the container. To start the RADI service just connect to
the running instance (view docker [command](#open-bash-connection-to-container) below) and run entrypoint.sh by hand or uncomment 
the entrypoint line before creating the instance.

```yaml
# ENTRYPOINT [ "./entrypoint.sh" ]
```
It also contains a running ssh server, so you can connect via ssh to the docker instance. It also allows to 
run ssh based development tools like Pycharm professional SSH development, which lets you run and debug code inside the
container. To view the docker instance IP address to connect to, just make sure the container instance is running
then run docker [inspect](#inspect-container-configuration) command and, at the end of the output you'll get container's
network configuration.

```commandline
            "Networks": {
                "bridge": {
                    "IPAMConfig": null,
                    "Links": null,
                    "Aliases": null,
                    "NetworkID": "72f49f71ca49230f7693a0e1ea32de75784ee1faee4b514ec5d01d4e900f0c84",
                    "EndpointID": "94cc2a25c2520c780658217f4eb22f9e74eb48e6f3868b481c87499b380adc21",
                    "Gateway": "172.17.0.1",
                    "IPAddress": "172.17.0.2",
                    "IPPrefixLen": 16,
                    "IPv6Gateway": "",
                    "GlobalIPv6Address": "",
                    "GlobalIPv6PrefixLen": 0,
                    "MacAddress": "02:42:ac:11:00:02",
                    "DriverOpts": null
                }
            }
```

Grab the IP address and use it in your ssh command. The username is *root* and the password is *password*. 
You can change the password in the DockerFile before building the instance.

```commandline
ssh root@172.17.0.2
```

## Useful docker commands

### container creation
```commandline
# creating container 
docker create -it --name test-ram -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7163:7163 -p 2022:22 jderobot/robotics-academy:new-ram-v0.2

# same but including local development folder as container one, to cross-develop inside container 
docker create -it --name test-ram -v /home/dmariaa/Desarrollo/unibotics/RoboticsAcademy:/RoboticsAcademy -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7163:7163 -p 2022:22 jderobot/robotics-academy:new-ram-v0.2
```

### starting container
```commandline
docker start test-ram
```

### stopping container
```commandline
docker stop test-ram
```

### removing container
```commandline
docker container rm test-ram
```

### open bash connection to container
```commandline
docker exec -it test-ram bash
```

### inspect container configuration
```commandline
docker inspect test-ram
```

## Exercise configuration inside Django management

```json
{
  "application": {
    "type": "python",
    "entry_point": "$EXERCISE_FOLDER/entry_point/exercise.py"
  },
  "launch": {
    "0": {
      "type": "module",
      "module": "ros_api",
      "resource_folders": [
        "$EXERCISE_FOLDER/launch"
      ],
      "model_folders": [
        "$CUSTOM_ROBOTS_BASE/f1/models"
      ],
      "plugin_folders": [
      ],
      "parameters": [],
      "launch_file": "$EXERCISE_FOLDER/launch/simple_line_follower_ros_headless_${circuit}.launch"
    }
  }
}
```