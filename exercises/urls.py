from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path("exercises/<slug:exercise_id>/", views.load_exercise, name="load_exercise"),
    path("exercise/request/<slug:exercise_id>", views.request_code, name="request_code"),
    path('evaluate_style/', views.evaluate_style, name="evaluate_style"),
    path('ros_version/', views.ros_version, name='ros_version'),
    path("exercise/<slug:exercise_id>/launch_files", views.launch_files, name='launch_files')
]
