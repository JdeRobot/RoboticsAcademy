from django.urls import path

from react_frontend import views
from django.views.generic import RedirectView

urlpatterns = [
    path("", views.academy, name="academy"),
    path("studio/<slug:proj_id>/", views.academy, name="academy"),
]
