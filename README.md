<a href="https://jderobot.github.io/"><img src="./img/logo.gif" width="150" align="right" /></a>

# RoboticsAcademy: Learn Robotics, Artificial Intelligence and Computer Vision

JdeRobot Academy is an **open source** platform that provides a collection of exercises for learning robotics in a practical way. Its latest documentation (including installation recipes, current available exercises and illustrative videos) is on its <a href="https://jderobot.github.io/RoboticsAcademy">webpage</a>. RoboticsAcademy is completely ROS-based, and includes robotics standard tools like Gazebo and Rviz.

If you are a contributor, please note that we use GitHub Pages and a Jekyll theme (MinimalMistakes) for the Academy web page. Feel free to install Jekyll locally, so that, you can test your changes before submitting your pull-request.

# Running RoboticsAcademy

```
#!/bin/bash

# Default: cpu and offline
gpu_mode="false"
nvidia="false"
base_path_offline="compose_cfg/"
compose_file="user_humble_cpu"
base_path_online="https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/humble-devel/compose_cfg/"

# Function to clean up the containers
cleanup() {
  echo "Cleaning up..."
  if [ "$nvidia" = "true" ]; then
    docker compose --compatibility down
  else
    docker compose down
  fi
  rm docker-compose.yaml
  
  exit 0
}

# Loop through the arguments using a while loop
while getopts ":g:n  " opt; do
  case $opt in
    g) gpu_mode="true" ;; 
    n) nvidia="true" ;;
    \?) echo "Invalid option: -$OPTARG" >&2 ;;   # If an invalid option is provided, print an error message
  esac
done

# Set up trap to catch interrupt signal (Ctrl+C) and execute cleanup function
trap 'cleanup' INT

# Set the compose file
if [ "$gpu_mode" = "true" ]; then
  compose_file="user_humble_gpu"
fi
if [ "$nvidia" = "true" ]; then
  compose_file="user_humble_nvidia"
fi

# Check the mode
if [ -d compose_cfg ]; then
  # Offline mode
  cp $base_path_offline$compose_file.yaml docker-compose.yaml
else
  # Online mode
  curl -sL $base_path_online$compose_file.yaml -o docker-compose.yaml
fi

# Execute docker compose
if [ "$nvidia" = "true" ]; then
  docker compose --compatibility up
else
  docker compose up
fi

cleanup```

# How to contribute?

Take a look at the [contributing](CONTRIBUTING.md) guide lines.

## Development resources

- [Instructions for developers.][]
- [Coding Style Guide][]
- [Client side.][] (Robotics Academy architecture) **Obsolete**
- [Repository Architecture.][]
- [Generate a RADI.][]
- [Publishing a RADI.][]
- [Humble RADI structure.][]
- [Troubleshooting Robotics Academy][]
- [Exercises Status][]

[Instructions for developers.]: ./docs/InstructionsForDevelopers.md
[Coding Style Guide]: ./docs/coding_style_guide.md
[Client side.]: ./docs/clientside.md
[Repository Architecture.]: ./docs/RepositoryArchitecture.md
[Generate a RADI.]: ./docs/generate_a_radi.md
[Publishing a RADI.]: ./.github/workflows/README.md
[Humble RADI structure.]: ./scripts/RADI/README.md
[Troubleshooting Robotics Academy]: ./docs/troubleshooting.md
[Exercises Status]: ./docs/exercise-status.md
