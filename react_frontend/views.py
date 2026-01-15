from django.shortcuts import render


# Create your views here.
def academy(request, proj_id=None):
    return render(request, "react_frontend/index.html")
