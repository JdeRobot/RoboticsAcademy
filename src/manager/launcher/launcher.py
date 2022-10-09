# Thanks to https://stackoverflow.com/questions/452969/does-python-have-an-equivalent-to-java-class-forname
# This should be moved to a utils library
def get_class(kls):
    parts = kls.split('.')
    module = ".".join(parts[:-1])
    m = __import__(module)
    for comp in parts[1:]:
        m = getattr(m, comp)
    return m


class Launcher:
    def __init__(self, config: dict):
        self.launcher = config
        self.module = '.'.join(__name__.split('.')[:-1])

    def run(self):
        keys = sorted(self.launcher.keys())
        for key in keys:
            launcher_data = self.launcher[key]
            launcher_type = launcher_data['type']

            if launcher_type == "module":
                self.launch_module(launcher_data)
            elif launcher_type == "command":
                self.launch_command(launcher_data)
            else:
                raise LauncherException(f"Launcher type {launcher_type} not valid")

    def launch_module(self, configuration):
        launcher_module_name = configuration["module"]
        launcher_module = f"{self.module}.launcher_{launcher_module_name}.Launcher{launcher_module_name.capitalize()}"
        launcher_class = get_class(launcher_module)
        launcher = launcher_class(configuration)
        process = launcher.run()

    def launch_command(self, configuration):
        pass


class LauncherException(Exception):
    def __init__(self, message):
        super(LauncherException, self).__init__(message)
