"""
models.py
"""

import json
import uuid
from django.db import models
import subprocess


StatusChoice = (
    ('ACTIVE', "ACTIVE"),
    ('INACTIVE', "INACTIVE"),
    ('PROTOTYPE', "PROTOTYPE")
)

VisualizationType = (
    ('none', "None"),
    ('console', "Console"),
    ('gazebo_gra', "Gazebo GRA"),
    ('gazebo_rae', "Gazebo RAE"),
    ('physic_gra', "Physic GRA"),
    ('physic_rae', "Physic RAE")
)

WorldType = (
    ('none', "None"),
    ('gazebo', "Gazebo"),
    ('drones', "Gazebo Drones"),
    ('physical', "Physical")
)

RosVersion = (
    ('ROS1', "ROS1"),
    ('ROS2', "ROS2")
)

# Create your models here.

class World(models.Model):
    name = models.CharField(max_length=40, default=uuid.uuid4, unique=True)
    ros_version = models.CharField(max_length=4, choices=RosVersion, default="none")
    world_type = models.CharField(
        max_length=20,
        choices=WorldType,
        default="none"
    )
    resource_folders = models.TextField(default="")
    model_folders = models.CharField(max_length=100, blank=False, default="$CUSTOM_ROBOTS_FOLDER/")
    launch_file = models.TextField(default=json.dumps({}))
    auxiliar_components = models.TextField(default=json.dumps({}))

    def __str__(self):
        return self.name

class Exercise(models.Model):
    """
    RoboticsCademy Exercise model
    """
    exercise_id = models.CharField(max_length=40, blank=False, unique=True)
    name = models.CharField(max_length=40, blank=False, unique=True)
    description = models.CharField(max_length=400, blank=False)
    assets = models.CharField(
        max_length=2000,
        default=json.dumps({"notebook": ""})
    )
    tags = models.CharField(
        max_length=2000,
        default=json.dumps({'tags': ""})
    )
    world_type = models.CharField(
        max_length=20,
        choices=WorldType,
        default="none"
    )
    worlds = models.ManyToManyField(World, default=None)
    status = models.CharField(
        max_length=20, choices=StatusChoice, default="ACTIVE")        
    visualization = models.CharField(
        max_length=20,
        choices=VisualizationType,
        default="none"
    )    
    
    configuration = models.TextField(default=json.dumps({}))

    def get_launch_files(self):
        data = {}
        for world in self.worlds.all():
            ros_version = world.ros_version
            launch_file = json.loads(world.launch_file)
            if ros_version not in data:
                data[ros_version] = []
            data[ros_version].append(launch_file)
        return data

    def get_resource_folders(self):
        data = {}
        for world in self.worlds.all():
            data[world.ros_version] = str(world.resource_folders)
        return data

    def get_model_folders(self):
        try:
            data = self.worlds.first().resource_folders
        except:
            data = ""
        return data
    
    def get_auxiliar_components(self):
        data = {}
        for world in self.worlds.all():
            ros_version = world.ros_version
            auxiliar_components = json.loads(world.auxiliar_components)
            if ros_version not in data:
                data[ros_version] = []
            data[ros_version].append(auxiliar_components)
        return data
    
    def __str__(self):
        return str(self.name)

    @property
    def context(self):
        """
        Build and return context
        """
        exercise_configuration = json.loads(self.configuration)
        resource_folders_dict = self.get_resource_folders()
        launch_files_dict = self.get_launch_files()
        auxiliar_components_dict = self.get_auxiliar_components()
        output = subprocess.check_output(['bash', '-c', 'echo $ROS_VERSION'])
        output_str = output.decode('utf-8')
        if output_str.strip() == '1':
            ros_version = 'ROS1'
        else:
            ros_version = 'ROS2'
        configurations = []

        if len(launch_files_dict[ros_version]) == len(exercise_configuration[ros_version]):
            for i in range(len(launch_files_dict[ros_version])):
                launch_file = launch_files_dict[ros_version][i]
                application_config = exercise_configuration[ros_version][i]                

                config = {
                    "launch": {},
                    "application": application_config["application"],
                    "exercise_id": str(self.exercise_id),
                    "visualization": self.visualization,
                    "world": self.world_type,
                    "resource_folders": resource_folders_dict[ros_version],
                    "model_folders": self.worlds.first().model_folders,
                    "launch_file": launch_files_dict[ros_version][i]["path"],
                    "name": launch_file["name"]
                }
                if ros_version in auxiliar_components_dict:
                    config.update({"launch": auxiliar_components_dict[ros_version][i]})
                configurations.append(config)  

            context = {'exercise_base': "exercise_base_2_RA.html",
                   'exercise_id': self.exercise_id,
                   'exercise_config': configurations,
                }
        return context
