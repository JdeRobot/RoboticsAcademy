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

VisualizationType = (
    ("none", "None"),
    ("console", "Console"),
    ("gazebo_gra", "Gazebo GRA"),
    ("gazebo_rae", "Gazebo RAE"),
    ("gzsim_gra", "Gz Sim GRA"),
    ("gzsim_rae", "Gz Sim RAE"),
    ("physic_gra", "Physic GRA"),
    ("physic_rae", "Physic RAE"),
)

UniverseType = (
    ("none", "None"),
    ("gazebo", "Gazebo"),
    ("drones", "Gazebo Drones"),
    ("gzsimdrones", "Gz Sim Drones"),
    ("physical", "Physical"),
)

RosVersion = (("ROS", "ROS"), ("ROS2", "ROS2"))


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
    visualization_config_path = models.CharField(max_length=200, blank=False)
    ros_version = models.CharField(max_length=4, choices=RosVersion, default="none")
    visualization = models.CharField(
        max_length=50, choices=VisualizationType, default="none", blank=False
    )
    world = models.CharField(
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
                        "world": universe.world.world,
                        "start_pose": universe.world.start_pose
                    }
                else:
                    robot_config = {
                        "name": None,
                        "launch_file_path": None,
                        "ros_version": None,
                        "world": None,
                        "start_pose": None,
                    }

                config = {
                    "name": universe.name,
                    "world": {
                        "name": universe.world.name,
                        "launch_file_path": universe.world.launch_file_path,
                        "ros_version": universe.world.ros_version,
                        "world": universe.world.world,
                    },
                    "visualization": universe.world.visualization,
                    "visualization_config_path": universe.world.visualization_config_path,
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
                    "world": None,
                },
                "robot": {
                    "name": None,
                    "launch_file_path": None,
                    "ros_version": None,
                    "world": None,
                    "start_pose": None,
                },
                "visualization": "console",
                "visualization_config_path": None,
                "template": self.template,
                "exercise_id": self.exercise_id,
            }
            configurations.append(config)

        context = {
            "exercise_base": "exercise_base_2_RA.html",
            "exercise_id": self.exercise_id,
            "exercise_config": configurations,
        }
        print(context)
        return context

    class Meta:
        db_table = '"exercises"'
