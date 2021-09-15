from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path("exercises/<slug:exercise_id>/", views.load_exercise, name="load_exercise"),
    path("exercise/request/<slug:exercise_id>", views.request_code, name="request_code"),
]
