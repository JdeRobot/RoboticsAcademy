"""
Django ORM models for Robotics Academy.

Defines the database schema for exercises, universes, worlds,
robots, and tools used by the Robotics Academy platform.
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
    Represents a tool available in the Robotics Academy platform.

    Attributes:
        name: Unique identifier and primary key for the tool.
        base_config: Default configuration string for the tool.
    """

    name = models.CharField(max_length=50, blank=False, unique=True, primary_key=True)
    base_config = models.CharField(max_length=200, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"tools"'


class Robot(models.Model):
    """
    Represents a robot available in the Robotics Academy platform.

    Attributes:
        name: Unique name identifying the robot.
        launch_file_path: Path to the ROS launch file for this robot.
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    launch_file_path = models.CharField(max_length=200, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"robots"'


class World(models.Model):
    """
    Represents a simulation world in the Robotics Academy platform.

    Attributes:
        name: Unique name identifying the world.
        launch_file_path: Path to the ROS launch file for this world.
        tools_config: JSON string with tool configuration overrides.
        ros_version: ROS version used (ROS or ROS2).
        type: Simulator type (none, gazebo, gz, physical).
        start_pose: List of starting poses for robots in this world.
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
    Represents a universe combining a world and a robot.

    Attributes:
        name: Unique name identifying the universe.
        world: Associated World instance.
        robot: Associated Robot instance.
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
    Represents a Robotics Academy exercise.

    Attributes:
        exercise_id: Unique string identifier for the exercise.
        name: Human-readable name of the exercise.
        description: Short description of the exercise goals.
        tags: JSON-encoded list of tags (e.g. MULTILANGUAGE).
        status: Lifecycle status (ACTIVE, INACTIVE, PROTOTYPE).
        universes: Associated Universe instances via ExerciseUniverses.
        tools: Associated Tool instances.
        url: Optional URL for additional exercise resources.
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
    """
    Through model linking Exercise and Universe with a default flag.

    Attributes:
        exercise: Related Exercise instance.
        universe: Related Universe instance.
        is_default: Whether this universe is the default for the exercise.
    """

    exercise = models.ForeignKey(Exercise, on_delete=models.CASCADE)
    universe = models.ForeignKey(Universe, on_delete=models.CASCADE)
    is_default = models.BooleanField()

    class Meta:
        db_table = '"exercises_universes"'
