from django.urls import path

from . import views

urlpatterns = [
    path("save_exercise_db/", views.save_exercise_db, name="save_exercise_db"),
    path("save_universe_db/", views.save_universe_db, name="save_universe_db"),
    path("get_info/", views.get_info, name="get_info"),
    path("get_exercise_list/", views.get_exercise_list, name="get_exercise_list"),
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
    ),
]
