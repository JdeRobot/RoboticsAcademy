from rest_framework import routers
from django.urls import path

from academy.academy_rest_api.views.exercises import ExerciseViewSet
from academy.academy_rest_api.views.exercises import FormatCode
from academy.academy_rest_api.views.exercises import CodeAnalysis

router = routers.SimpleRouter()
router.register(r'exercises', ExerciseViewSet)

urlpatterns = router.urls +[
    path('format/', FormatCode.as_view(), name='format_code'),
    path('analysis/', CodeAnalysis.as_view(), name='analysis_code'),
]
