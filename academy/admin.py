from django.contrib import admin
from .models import Exercise, Scene, World, Tool, Robot

# Register your models here.


class SaveExAdmin(admin.ModelAdmin):
    change_list_template = "./admin/change_list_ex.html"


class SaveWorldAdmin(admin.ModelAdmin):
    change_list_template = "./admin/change_list_world.html"


admin.site.register(Exercise, SaveExAdmin)
admin.site.register(Tool, SaveWorldAdmin)
admin.site.register(World, SaveWorldAdmin)
admin.site.register(Scene, SaveWorldAdmin)
admin.site.register(Robot, SaveWorldAdmin)
