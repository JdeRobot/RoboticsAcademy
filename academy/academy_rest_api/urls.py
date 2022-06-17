from rest_framework import routers

from academy.academy_rest_api.views.exercises import ExerciseViewSet

router = routers.SimpleRouter()
router.register(r'exercises', ExerciseViewSet)

urlpatterns = router.urls
