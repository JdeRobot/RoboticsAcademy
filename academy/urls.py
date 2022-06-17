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

urlpatterns = [
    path('exercises/', include('react_frontend.urls')),

    path('admin/', admin.site.urls),
    path('exercises/', include('exercises.urls')),
    # path('', include('exercises.urls')),
    path('', lambda request: redirect('exercises/', permanent=False)),

    # rest api url
    path('api/v1/', include('academy.academy_rest_api.urls')),
]
