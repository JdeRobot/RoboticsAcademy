<a href="https://jderobot.github.io/"><img src="./img/logo.gif" width="150" align="right" /></a>

# RoboticsAcademy: Learn Robotics, Artificial Intelligence and Computer Vision

JdeRobot Academy is an **open source** platform that provides a collection of exercises for learning robotics in a practical way. Its latest documentation (including installation recipes, current available exercises and illustrative videos) is on its <a href="https://jderobot.github.io/RoboticsAcademy">webpage</a>. RoboticsAcademy is completely ROS-based, and includes robotics standard tools like Gazebo and Rviz. :contentReference[oaicite:0]{index=0}

If you are a contributor, please note that we use GitHub Pages and a Jekyll theme (MinimalMistakes) for the Academy web page. Feel free to install Jekyll locally, so that you can test your changes before submitting your pull request.

# Running RoboticsAcademy

Run the following command to download and start the RoboticsAcademy environment:

```
curl -s https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/humble-devel/scripts/run_academy.sh | sudo bash
```

This command downloads and executes the `run_academy.sh` script, which automatically sets up and launches the RoboticsAcademy Docker environment with all the required dependencies such as ROS, Gazebo, and other robotics tools. The containerized environment simplifies installation by providing a ready-to-run robotics backend. :contentReference[oaicite:1]{index=1}

# How to contribute?

Take a look at the [contributing](CONTRIBUTING.md) guidelines.

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