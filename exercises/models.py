"""
models.py
"""

import json
from django.db import models


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

# Create your models here.

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
    status = models.CharField(
        max_length=20,
        choices=StatusChoice,
        default="ACTIVE"
    )
    world = models.CharField(
        max_length=20,
        choices=WorldType,
        default="none"
    )
    resource_folders = models.TextField(default=json.dumps({}))
    model_folders = models.CharField(max_length=100, blank=False, default="$CUSTOM_ROBOTS_FOLDER/")
    launch_files = models.TextField(default=json.dumps({}))
    visualization = models.CharField(
        max_length=20,
        choices=VisualizationType,
        default="none"
    )    
    
    configuration = models.TextField(default=json.dumps({}))

    def __str__(self):
        return str(self.name)

    @property
    def context(self):
        """
        Build and return context
        """
        exercise_configuration = json.loads(self.configuration)

        # extend exercise configuration with some useful stuff
        # TODO: Review if there's a better way
        exercise_configuration["exercise_id"] = self.exercise_id
        exercise_configuration["world"] = self.world
        exercise_configuration["resource_folders"] = self.resource_folders
        exercise_configuration["model_folders"] = self.model_folders
        exercise_configuration["launch_files"] = self.launch_files
        exercise_configuration["visualization"] = self.visualization

        # compatibility code for old assets field
        # TODO: Remove if not needed
        exercise_assets = json.loads(self.assets)

        # compatibility context
        context = {'exercise_base': "exercise_base_2_RA.html",
                   'exercise_id': self.exercise_id,
                   'exercise_config': exercise_configuration,
                   'indexs': exercise_assets.get('indexs', []),
                   'statics': exercise_assets.get('statics', [])}

        return context
