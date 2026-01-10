from django.conf import settings


def version(request):
    """
    Context processor to expose the system version to templates.

    This function makes the application version available globally
    in Django templates via the context.

    Parameters
    ----------
    request : HttpRequest
        The incoming HTTP request object.

    Returns
    -------
    dict
        A dictionary containing the system version under the
        key 'SYS_VERSION'.
    """
    return {"SYS_VERSION": settings.VERSION}
