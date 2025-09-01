from django.test import TestCase
from exercises.models import Exercise, Universe, World, Robot, Tool
import json

class ExerciseModelTest(TestCase):
    @classmethod
    def setUpTestData(cls):
        # Create test data that doesn't modify the production database
        tool = Tool.objects.create(name="test_tool", base_config="None")
        robot = Robot.objects.create(name="test_robot", launch_file_path="/path/to/launch")
        world = World.objects.create(
            name="test_world",
            launch_file_path="/path/to/world",
            tools_config="None",
            ros_version="ROS2",
            type="gazebo",
            start_pose=[[0.0, 0.0, 0.0]]
        )
        universe = Universe.objects.create(name="test_universe", world=world, robot=robot)
        
        exercise = Exercise.objects.create(
            exercise_id="test_exercise",
            name="Test Exercise",
            description="Test description",
            tags=json.dumps({"tags": ["ROS2", "test"]}),
            status="ACTIVE"
        )
        exercise.universes.add(universe)
        exercise.tools.add(tool)

    def test_exercise_creation(self):
        exercise = Exercise.objects.get(exercise_id="test_exercise")
        self.assertEqual(exercise.name, "Test Exercise")
        self.assertEqual(exercise.status, "ACTIVE")

    def test_exercise_context(self):
        exercise = Exercise.objects.get(exercise_id="test_exercise")
        context = exercise.context
        self.assertIn("exercise_base", context)
        self.assertEqual(context["exercise_id"], "test_exercise")
        self.assertIsInstance(context["exercise_config"], list)

class UniverseModelTest(TestCase):
    @classmethod
    def setUpTestData(cls):
        robot = Robot.objects.create(name="test_robot", launch_file_path="/path/to/launch")
        world = World.objects.create(
            name="test_world",
            launch_file_path="/path/to/world",
            tools_config="None",
            ros_version="ROS2",
            type="gazebo",
            start_pose=[[0.0, 0.0, 0.0]]
        )
        Universe.objects.create(name="test_universe", world=world, robot=robot)

    def test_universe_creation(self):
        universe = Universe.objects.get(name="test_universe")
        self.assertEqual(universe.world.name, "test_world")
        self.assertEqual(universe.robot.name, "test_robot")

class WorldModelTest(TestCase):
    @classmethod
    def setUpTestData(cls):
        World.objects.create(
            name="test_world",
            launch_file_path="/path/to/world",
            tools_config="None",
            ros_version="ROS2",
            type="gazebo",
            start_pose=[[0.0, 0.0, 0.0]]
        )

    def test_world_creation(self):
        world = World.objects.get(name="test_world")
        self.assertEqual(world.launch_file_path, "/path/to/world")
        self.assertEqual(world.ros_version, "ROS2")
        self.assertEqual(world.type, "gazebo")
        self.assertEqual(world.start_pose, [[0.0, 0.0, 0.0]])

class RobotModelTest(TestCase):
    @classmethod
    def setUpTestData(cls):
        Robot.objects.create(name="test_robot", launch_file_path="/path/to/launch")

    def test_robot_creation(self):
        robot = Robot.objects.get(name="test_robot")
        self.assertEqual(robot.launch_file_path, "/path/to/launch")

class ToolModelTest(TestCase):
    @classmethod
    def setUpTestData(cls):
        Tool.objects.create(name="test_tool", base_config="None")

    def test_tool_creation(self):
        tool = Tool.objects.get(name="test_tool")
        self.assertEqual(tool.base_config, "None")
