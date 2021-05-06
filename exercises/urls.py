from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'), path("exercises/<slug:exercise_id>/", views.load_exercise, name="load exercise"),
]
