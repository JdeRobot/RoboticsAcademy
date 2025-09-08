class InvalidPath(Exception):
    """Exception raised when a path is not valid."""

    def __init__(self, msg):
        self.message = f"Pat: {msg} is invalid."
        super().__init__(self.message)
        self.error_code = 403

    def __str__(self):
        return f"{self.message}"


class ParameterInvalid(Exception):
    """Exception raised when a parameter is not passed correctly."""

    def __init__(self, msg):
        self.message = f"Parameter {msg} is required."
        super().__init__(self.message)
        self.error_code = 400

    def __str__(self):
        return f"{self.message}"


class ResourceAlreadyExists(Exception):
    """Exception raised for finding a resource that already exists."""

    def __init__(self, msg):
        self.message = f"{msg} already exists"
        super().__init__(self.message)
        self.error_code = 409

    def __str__(self):
        return f"{self.message}"


class ResourceNotExists(Exception):
    """Exception raised for not finding a resource."""

    def __init__(self, msg):
        self.message = f"{msg} does not exist"
        super().__init__(self.message)
        self.error_code = 404

    def __str__(self):
        return f"{self.message}"
