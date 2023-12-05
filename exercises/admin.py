from django.contrib import admin
from django.db import models
from .models import Exercise, World


# Register your models here.

admin.site.register(Exercise)
admin.site.register(World)
