from django.shortcuts import render
from .models import Exercise

# Create your views here.
def index(request):
	exercises = Exercise.objects.all()
	context = { "exercises": exercises }	
	return render(request, 'exercises/RoboticsAcademy.html', context)

def load_exercise(request, exercise_id):
	context = { "exercise": str(exercise_id) }
	return HttpResponse(template.render(context, request))    
