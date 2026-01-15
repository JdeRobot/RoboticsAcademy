from django.urls import path

from react_frontend import views

urlpatterns = [
    # path('', views.exercises, name='exercises'),
    path("", views.academy, name="academy")
]
