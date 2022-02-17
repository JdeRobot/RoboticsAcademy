from django.db import models
import json


# Create your models here.

class Exercise(models.Model):
    exercise_id = models.CharField(max_length=40, blank=False, unique=True)
    name = models.CharField(max_length=40, blank=False, unique=True)
    description = models.CharField(max_length=400, blank=False)
    assets = models.CharField(max_length=2000, default=json.dumps({"notebook": ""}))  # JSON Object.

    def __str__(self):
        return self.name
