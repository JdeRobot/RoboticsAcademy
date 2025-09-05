from django.urls import path

from . import views

urlpatterns = [
    path("", views.index, name="index"),
    path("exercises/<slug:exercise_id>/", views.load_exercise, name="load_exercise"),
    path(
        "exercise/<slug:exercise_id>/user_code_zip",
        views.user_code_zip,
        name="user_code_zip",
    ),
    path(
        "user_code_zip/",
        views.user_code_zip,
        name="user_code_zip",
    ),
    path(
        "get_universes_list/",
        views.get_universes_list,
        name="get_universes_list",
    ),
    path(
        "get_docker_universe_data/",
        views.get_docker_universe_data,
        name="get_docker_universe_data",
    )
]
