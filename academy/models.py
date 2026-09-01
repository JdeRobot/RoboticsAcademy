"""
Django ORM models for Robotics Academy.

Defines the database schema for exercises, scenes, worlds,
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
    entity = models.CharField(max_length=32, blank=False)
    extra_config = models.CharField(max_length=256, blank=False)
    model_path = models.CharField(max_length=200, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"robots"'


class Scene(models.Model):
    """
    Represents a simulation scene in the Robotics Academy platform.

    Attributes:
        name: Unique name identifying the scene.
        launch_file_path: Path to the ROS launch file for this scene.
        tools_config: JSON string with tool configuration overrides.
        ros_version: ROS version used (ROS or ROS2).
        type: Simulator type (none, gz, physical).
        start_pose: List of starting poses for robots in this scene.
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    launch_file_path = models.CharField(max_length=200, blank=False)
    tools_config = models.CharField(max_length=200, blank=False)
    ros_version = models.CharField(max_length=4, choices=RosVersion, default="none")
    type = models.CharField(
        max_length=50, choices=UniverseType, default="none", blank=False
    )
    model_path = models.CharField(max_length=200, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"scenes"'


class World(models.Model):
    """
    Represents a world combining a scene and one or more robots.

    Attributes:
        name: Unique name identifying the world.
        scene: Associated Scene instance.
        robots: Associated Robots instances.
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    scene = models.OneToOneField(
        Scene, default=None, on_delete=models.CASCADE, db_column="scene_id"
    )
    robots = models.ManyToManyField(Robot, default=None, through="WorldRobots")

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"worlds"'


# Create your models here.


class Exercise(models.Model):
    """
    Represents a Robotics Academy exercise.

    Attributes:
        exercise_id: Unique string identifier for the exercise.
        name: Human-readable name of the exercise.
        description: Short description of the exercise goals.
        tags: JSON-encoded list of tags (e.g. MULTILANGUAGE).
        entrypoints: JSON-encoded list of additional entrypoints for MULTI-ENTRYPOINT.
        status: Lifecycle status (ACTIVE, INACTIVE, PROTOTYPE).
        worlds: Associated Universe instances via ExerciseWorlds.
        tools: Associated Tool instances.
        url: Optional URL for additional exercise resources.
    """

    exercise_id = models.CharField(max_length=40, blank=False, unique=True)
    name = models.CharField(max_length=40, blank=False, unique=True)
    description = models.CharField(max_length=400, blank=False)
    tags = models.CharField(max_length=2000, default=[])
    entrypoints = models.CharField(max_length=2000, default=[])
    status = models.CharField(max_length=20, choices=StatusChoice, default="ACTIVE")
    worlds = models.ManyToManyField(World, default=None, through="ExerciseWorlds")
    tools = models.ManyToManyField(Tool, default=None, db_table='"exercises_tools"')
    url = models.CharField(max_length=200, blank=True, default="")

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"exercises"'


class ExerciseWorlds(models.Model):
    """
    Through model linking Exercise and World with a default flag.

    Attributes:
        exercise: Related Exercise instance.
        world: Related World instance.
        is_default: Whether this world is the default for the exercise.
    """

    exercise = models.ForeignKey(Exercise, on_delete=models.CASCADE)
    world = models.ForeignKey(World, on_delete=models.CASCADE)
    is_default = models.BooleanField()

    class Meta:
        db_table = '"exercises_worlds"'


class WorldRobots(models.Model):
    """
    Through model linking World and Robot with the number of instances of said robot.

    Attributes:
        world: Related World instance.
        robot: Related Robot instance.
        instances: Number of instances of the robot in a world (defaults to 1).
    """

    world = models.ForeignKey(World, on_delete=models.CASCADE)
    robot = models.ForeignKey(Robot, on_delete=models.CASCADE)
    poses = ArrayField(
        ArrayField(
            models.DecimalField(
                decimal_places=4, max_digits=10, default=None, blank=False
            )
        )
    )

    class Meta:
        db_table = '"worlds_robots"'
