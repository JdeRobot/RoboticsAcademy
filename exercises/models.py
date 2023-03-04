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
