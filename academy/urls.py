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

urlpatterns = [
    path("admin/", admin.site.urls),
    path("academy/", include("react_frontend.urls")),
    path("academy/", include("exercises.urls")),
    path("", lambda request: redirect("academy/", permanent=False)),
    path("save_exercise_db/", views.save_exercise_db, name="save_exercise_db"),
    path("save_universe_db/", views.save_universe_db, name="save_universe_db"),
]
