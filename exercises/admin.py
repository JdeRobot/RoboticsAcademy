from django.contrib import admin
from .models import Exercise, Universe, World, Tool, Robot


# Register your models here.

admin.site.register(Exercise)
admin.site.register(Tool)
admin.site.register(Universe)
admin.site.register(World)
admin.site.register(Robot)
