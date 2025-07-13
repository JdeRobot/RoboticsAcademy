model_dir_path = "/workspace/code/model.onnx"


# This file is part of the Human Detection ROS2 package.
def model_path_func() -> str:
    return model_dir_path


model_path = model_path_func()
