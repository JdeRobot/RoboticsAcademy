from django.test import TestCase, Client
from django.urls import reverse
from exercises.models import Exercise, Universe, World, Robot, Tool
import json

class ExerciseViewTest(TestCase):
    @classmethod
    def setUpTestData(cls):
        # Create test data
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

    def test_exercise_list_view(self):
        client = Client()
        response = client.get(reverse('exercises:exercise_list'))
        self.assertEqual(response.status_code, 200)
        self.assertContains(response, "Test Exercise")

    def test_exercise_detail_view(self):
        client = Client()
        response = client.get(reverse('exercises:exercise_detail', args=['test_exercise']))
        self.assertEqual(response.status_code, 200)
        self.assertContains(response, "Test Exercise")
