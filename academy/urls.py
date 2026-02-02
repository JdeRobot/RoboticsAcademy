"""academy URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/3.2/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""

from django.contrib import admin
from django.shortcuts import redirect
from django.urls import include, path
from . import views

academy_urls = [
    path("get_info/", views.get_info, name="get_info"),
    path("get_exercise_list/", views.get_exercise_list, name="get_exercise_list"),
    path(
        "get_helper_file_list/", views.get_helper_file_list, name="get_helper_file_list"
    ),
    path("get_helper_file/", views.get_helper_file, name="get_helper_file"),
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
    # File Management
    path("create_file/", views.create_file, name="create_file"),
    path("delete_file/", views.delete_file, name="delete_file"),
    path("rename_file/", views.rename_file, name="rename_file"),
    path("save_file/", views.save_file, name="save_file"),
    path("get_file/", views.get_file, name="get_file"),
    path("get_file_list/", views.get_file_list, name="get_file_list"),
    # Folder Management
    path("create_folder/", views.create_folder, name="create_folder"),
    path("delete_folder/", views.delete_folder, name="delete_folder"),
    path("rename_folder/", views.rename_folder, name="rename_folder"),
    path("upload/", views.upload, name="upload"),
]

urlpatterns = [
    path("", lambda request: redirect("academy/", permanent=False)),
    path("admin/", admin.site.urls),
    path("academy/", include("react_frontend.urls")),
    path("academy/", include(academy_urls)),
    path("save_exercise_db/", views.save_exercise_db, name="save_exercise_db"),
    path("save_universe_db/", views.save_universe_db, name="save_universe_db"),
]
