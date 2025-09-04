"""
models.py
"""

import json
from django.db import models
from django.contrib.postgres.fields import ArrayField
import subprocess

StatusChoice = (
    ("ACTIVE", "ACTIVE"),
    ("INACTIVE", "INACTIVE"),
    ("PROTOTYPE", "PROTOTYPE"),
)

UniverseType = (
    ("none", "None"),
    ("gazebo", "Gazebo"),
    ("gz", "Gazebo Harmonic"),
    ("physical", "Physical"),
)

RosVersion = (("ROS", "ROS"), ("ROS2", "ROS2"))


class Tool(models.Model):
    """
    Modelo Tool para Robotics Academy
    """

    name = models.CharField(max_length=50, blank=False, unique=True, primary_key=True)
    base_config = models.CharField(max_length=200, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"tools"'


class Robot(models.Model):
    """
    Modelo Robot para RoboticsAcademy
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    launch_file_path = models.CharField(max_length=200, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"robots"'


class World(models.Model):
    """
    Modelo World para RoboticsCademy
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    launch_file_path = models.CharField(max_length=200, blank=False)
    tools_config = models.CharField(max_length=200, blank=False)
    ros_version = models.CharField(max_length=4, choices=RosVersion, default="none")
    type = models.CharField(
        max_length=50, choices=UniverseType, default="none", blank=False
    )

    start_pose = ArrayField(
        ArrayField(
            models.DecimalField(
                decimal_places=4, max_digits=10, default=None, blank=False
            )
        )
    )

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"worlds"'


class Universe(models.Model):
    """
    Modelo Universe para Robotics Academy
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    world = models.OneToOneField(
        World, default=None, on_delete=models.CASCADE, db_column='"world_id"'
    )
    robot = models.OneToOneField(
        Robot, default=None, on_delete=models.CASCADE, db_column='"robot_id"'
    )

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"universes"'


# Create your models here.


class Exercise(models.Model):
    """
    Robotics Academy Exercise model
    """

    exercise_id = models.CharField(max_length=40, blank=False, unique=True)
    name = models.CharField(max_length=40, blank=False, unique=True)
    description = models.CharField(max_length=400, blank=False)
    tags = models.CharField(max_length=2000, default=json.dumps({"tags": ""}))
    status = models.CharField(max_length=20, choices=StatusChoice, default="ACTIVE")
    universes = models.ManyToManyField(
        Universe, default=None, db_table='"exercises_universes"'
    )
    tools = models.ManyToManyField(Tool, default=None, db_table='"exercises_tools"')
    template = models.CharField(max_length=200, blank=True, default="")

    def __str__(self):
        return str(self.name)

    @property
    def context(self):
        """
        Build and return context
        """
        configurations = []

        output = subprocess.check_output(["bash", "-c", "echo $ROS_VERSION"])
        output_str = output.decode("utf-8")
        if output_str.strip() == "2":
            ros_version = "ROS2"
        else:
            ros_version = "ROS"

        tools = []
        tools_config = {}
        for tool in self.tools.all():
            tools.append(tool.name)
            if tool.base_config != "None":
                tools_config.update({tool.name: tool.base_config})

        for universe in self.universes.all():
            if (
                universe.world.ros_version == ros_version
                and universe.world.name != "None"
            ):
                if universe.robot.name != "None":
                    robot_config = {
                        "name": universe.robot.name,
                        "launch_file_path": universe.robot.launch_file_path,
                        "ros_version": universe.world.ros_version,
                        "type": universe.world.type,
                        "start_pose": universe.world.start_pose,
                    }
                else:
                    robot_config = {
                        "name": None,
                        "launch_file_path": None,
                        "ros_version": None,
                        "type": None,
                        "start_pose": None,
                    }

                tools_configuration = None
                if universe.world.tools_config != "None":
                    tools_configuration = json.loads(universe.world.tools_config)

                config = {
                    "name": universe.name,
                    "world": {
                        "name": universe.world.name,
                        "launch_file_path": universe.world.launch_file_path,
                        "ros_version": universe.world.ros_version,
                        "type": universe.world.type,
                        "tools_config": tools_configuration,
                    },
                    "tools": tools,
                    "tools_config": tools_config,
                    "robot": robot_config,
                    "template": self.template,
                    "exercise_id": self.exercise_id,
                }

                configurations.append(config)

        # If empty worlds add one by default
        if len(configurations) == 0:
            config = {
                "name": None,
                "world": {
                    "name": None,
                    "launch_file_path": None,
                    "ros_version": None,
                    "type": None,
                    "tools_config": None,
                },
                "robot": {
                    "name": None,
                    "launch_file_path": None,
                    "ros_version": None,
                    "type": None,
                    "start_pose": None,
                },
                "tools": tools,
                "tools_config": tools_config,
                "template": self.template,
                "exercise_id": self.exercise_id,
            }
            configurations.append(config)

        # Accesible from the exercise using document.getElementById("exercise-data")
        context = {
            "exercise_data": {
                "universes": configurations,
                "tools": tools,
                "name": self.name,
            },
        }
        return context

    class Meta:
        db_table = '"exercises"'
