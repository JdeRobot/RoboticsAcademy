from django.shortcuts import render
from django.views.decorators.cache import never_cache


# Create your views here.
@never_cache
def academy(request, proj_id=None):
    return render(request, "react_frontend/index.html")
