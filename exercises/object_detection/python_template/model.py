import os

model_dir_path = "/workspace/code"

def model_path_func(name: str) -> str:
    return os.path.join(model_dir_path, name)