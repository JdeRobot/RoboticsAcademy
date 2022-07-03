from django.shortcuts import render


# Create your views here.
def exercises(request):
    return render(request, 'react_frontend/index.html')


def exercise(request):
    pass
