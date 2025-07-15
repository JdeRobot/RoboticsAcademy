from django.urls import path

from . import views

urlpatterns = [
    path("", views.index, name="index"),
    path("exercises/<slug:exercise_id>/", views.load_exercise, name="load_exercise"),
    path("exercise/<slug:exercise_id>/user_code_zip", views.user_code_zip, name='user_code_zip'),
]
