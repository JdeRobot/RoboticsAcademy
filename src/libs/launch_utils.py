# Thanks to https://stackoverflow.com/questions/452969/does-python-have-an-equivalent-to-java-class-forname
# This should be moved to a utils library
def get_class(kls):
    parts = kls.split('.')
    module = ".".join(parts[:-1])
    m = __import__(module)
    for comp in parts[1:]:
        m = getattr(m, comp)
    return m


def class_from_module(module: str):
    """
    Capitalizes a module name to create class name
    """
    return ''.join([s.capitalize() for s in module.split('_')])
