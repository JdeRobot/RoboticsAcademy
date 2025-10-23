from django.contrib import admin
from .models import Exercise, Universe, World, Tool, Robot

from django.contrib import admin

# Register your models here.

class SaveExAdmin(admin.ModelAdmin):
    change_list_template = './admin/change_list_ex.html'

class SaveUnivAdmin(admin.ModelAdmin):
    change_list_template = './admin/change_list_univ.html'

admin.site.register(Exercise, SaveExAdmin)
admin.site.register(Tool, SaveUnivAdmin)
admin.site.register(Universe, SaveUnivAdmin)
admin.site.register(World, SaveUnivAdmin)
admin.site.register(Robot, SaveUnivAdmin)