from django.db import models
import json

# Create your models here.

class Exercise(models.Model):
    exercise_id = models.CharField(max_length=40, blank=False, unique=True)
    name = models.CharField(max_length=40, blank=False, unique=True)

    STATE = (
        ('active', 'Active'),
        ('inactive', 'Inactive'),
        ('testing', 'Testing')
    )
    state = models.CharField(max_length=40, blank=False, choices=STATE)

    PLATFORM = (
        ('gazebo', 'Gazebo'),
        ('real', 'Real'),
        ('theory', 'Theory'),
        ('vision', 'Vision'),
        ('tutorials', 'Tutoriales')
    )
    platform = models.CharField(max_length=40, blank=True, choices=PLATFORM)

    LANGUAGE = {
        ('python', 'Python'),
        ('javascript', 'JavaScript'),
        ('scratch', 'Scratch')
    }
    language = models.CharField(max_length=40, choices=LANGUAGE)
    description = models.CharField(max_length=400, blank=False)
    video = models.CharField(max_length=100, blank=True)
    topic = models.CharField(max_length=100, blank=True)
    thumbnail = models.CharField(max_length=100, blank=True)
    assets = models.CharField(max_length=2000, default=json.dumps({"notebook": ""}))  # JSON Object.
    gui = models.CharField(max_length=2000, blank=True)  # Estructura {"client":"", "server":""}
    referee = models.CharField(max_length=2000, blank=True)  # Estructura {"client":"", "server":""}
    real = models.CharField(max_length=50, blank=True)  # vista para la descarga en real
    local = models.CharField(max_length=2000, blank=True)
    compute_load = models.IntegerField(blank=False, default=100)  # Unidades: Unidad de Computo (UC)
    observations = models.TextField(max_length=500, blank=True)

    def __unicode__(self):  # __unicode__ for Python 2
        return self.name

    def get_observations(self):
        if self.local:
            return json.loads(self.local)
        else:
            return None

    def exercise_location(self, local=False):
        """ Ruta a la carpeta que contiene los archivos propios de cada ejercicio """
        if local:
            return settings.EXERCISES_DIR + '/' + self.language + "/" + self.exercise_id + '/local/'
        else:
            return settings.EXERCISES_DIR + '/' + self.language + "/" + self.exercise_id + '/'

    def exercise_file_location(self, local=False):
        """ Ruta al archivo con el código base (cuadernillo por defecto) del ejercicio """
        assets = json.loads(self.assets)
        if local:
            return settings.BASE_DIR + "/exercises/" + self.language + "/" + self.exercise_id + "/local/" + 'local_' + \
                   assets["notebook"]
        else:
            return settings.BASE_DIR + "/exercises/" + self.language + "/" + self.exercise_id + "/" + assets["notebook"]
