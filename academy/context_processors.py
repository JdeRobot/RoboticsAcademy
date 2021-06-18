from django.conf import settings

def version(request):
    return {
        'SYS_VERSION': settings.VERSION
    }