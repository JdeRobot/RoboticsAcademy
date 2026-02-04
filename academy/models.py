"""
models.py
"""

from django.db import models
from django.contrib.postgres.fields import ArrayField

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
    tags = models.CharField(max_length=2000, default=[])
    status = models.CharField(max_length=20, choices=StatusChoice, default="ACTIVE")
    universes = models.ManyToManyField(
        Universe, default=None, through="ExerciseUniverses"
    )
    tools = models.ManyToManyField(Tool, default=None, db_table='"exercises_tools"')
    url = models.CharField(max_length=200, blank=True, default="")

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"exercises"'


class ExerciseUniverses(models.Model):
    exercise = models.ForeignKey(Exercise, on_delete=models.CASCADE)
    universe = models.ForeignKey(Universe, on_delete=models.CASCADE)
    is_default = models.BooleanField()

    class Meta:
        db_table = '"exercises_universes"'
