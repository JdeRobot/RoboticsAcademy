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

    @property
    def context(self):
        exercise_assets = json.loads(self.assets)

        # compatibility context
        context = {'exercise_base': "exercise_base_2_RA.html",
                   'exercise_id': self.exercise_id,
                   'indexs': exercise_assets.get('indexs', []),
                   'statics': exercise_assets.get('statics', [])}

        exercise_assets.pop('indexs', None)
        exercise_assets.pop('statics', None)

        # configuration context
        context['exercise_config'] = exercise_assets

        return context
