---
permalink: /developer_guide
title: "Contributor Guide"


sidebar:
  nav: "docs"

toc: true
toc_label: "TOC installation"
toc_icon: "cog"
---

This page contains information useful to understand how to contribute to Robotics Academy: creating a new exercise, improving the documentation, fixing bugs... Become a developer, not only a Robotics Academy user :-)


# Roadmap

1. Migrate all exercises to Robotics-Academy 4 release (ROS2 Foxy)
2. Develop new exercises
    + map building: occupancy grid
    + localization algorithm with particle filter and laser sensor
    + Computer Vision exercises
4. Improve RADI for ROS1-Noetic and ROS2-Foxy


# Releases
* 3.3
    + Browser for edition and execution
    + REACT based frontend
    + Robotics Application Manager 
    + based in ROS-Noetic and Gazebo11
* 3.2
    + Browser for edition and execution
    + based in ROS-Noetic and Gazebo11
* 2.3
    + Web pages for exercise templates
    + Browser for editing user code and execution monitoring
    + Dependencies are pre-installed in [RADI Docker Image](https://hub.docker.com/r/jderobot/robotics-academy/tags)
    + based in ROS-Melodic and Gazebo9
    + works on Linux, Windows and MacOS
* 2.1
    + ROS nodes for exercise templates
    + File editing for user code
    + Dependencies should be installed locally, debian and ROS packages
    + based in ROS-Melodic and Gazebo9
* 2.0


# Developer documentation

Check all the documents at the [docs directory at Robotics Academy repository](https://github.com/JdeRobot/RoboticsAcademy/tree/master/docs)

# Creating a new exercise

Just use [Robotics Academy Discussions](https://github.com/JdeRobot/RoboticsAcademy/discussions) to propose a new exercise to be included. Also contact us at *jderobot@gmail.com*



# Adding or improving the user documentation for an exercise?

The user documentation for each exercise has to be in GitHub Pages [gh-pages branch at the RoboticsAcademy repository](https://github.com/JdeRobot/RoboticsAcademy/tree/gh-pages), in markdown format. By default, the information added to the documentation is split in two:

- A `.md` file with the description of the exercise, steps for execution, comments, etc.
- A folder with resources. Typically images and videos that are associated to the exercise.

The file with the information is placed in the path: `/_pages/exercises/<section>/<exercise_name>/files.md` with the name of the exercise and with Markdown extension (`.md`) inside a folder with the same name.

The images associated to this file with the information are stored in the path: `/assets/images/exercises/<exercise_name>/`

## Name policy

The name policy is simple:

All file names attached to the documentation will be **lowercase** and spaces will be replaced by **underscore** ("`_`") using ASCII characters.

## Images policy

In order to maintain coherence between all the exercises, it is necessary to distinguish between one image and the rest. This is the image shown in the set of exercises.

The image name policy for teaser is `<exercise_name>_teaser`. (Note how it has to end in `_teaser`). For teaser images the required size has to be multiple $600x400$ px.

The rest of the images have no name restriction. When in doubt, they are named as the exercise and with a number behind them.

- For the images of the exercises or menus use the [name of the Minimal Mistakes gallery](https://mmistakes.github.io/minimal-mistakes/post%20formats/post-gallery/).


## Exercise points

All exercises will contain the following points:

1. **Goal**: Brief description of the objective of the exercise
2. **Installation**: If the exercise requires an additional installation, the additional commands are added in this section.
3. **How to perform the exercise?**: The files to be modified to solve the exercise are defined as well as the functions available to obtain information from the sensors and send to the actuators.
4. **How to run your solution?**: The commands and settings to be launched to test the developed code are specified
5. **Theory**: The information about the exercise is described **in detail**. 
6. **Hints**: Clues are given as to where the exercise can be tackled.
7. **Demonstrate Video**:  Video with progress, clues and/or the result of the exercise
8. **Contributors**: Two groups are described: contributors (all the original authors who developed the exercise code) and the maintainer (the person responsible for the exercise, updating it, correcting errors, resolving doubts, etc, is the contributor who knows the exercise best at present). 
9. **References**: Information on the points described in the theory section, articles, websites with information, etc.

*Note: If at any point in the exercise description it is required to add mathematical equations, these must be put into the equation format in LaTeX. You can use this [online equation generator](https://www.hostmath.com/) for ease of use. For more information about equations in LaTeX, check their [documentation](https://en.wikibooks.org/wiki/LaTeX/Mathematics). *



## Jekyll Local Installation

### Installing Ruby on Ubuntu

First of all, we need to install all the dependencies typing:

```bash
sudo apt-get install ruby-full build-essential zlib1g-dev
```

After that, we need to set up a gem installation directory for your user account. The following commands will add environment variables to your `~/.bashrc` file to configure the gem installation path. Run them now:

```bash
echo '# Install Ruby Gems to ~/gems' >> ~/.bashrc
echo 'export GEM_HOME="$HOME/gems"' >> ~/.bashrc
echo 'export PATH="$HOME/gems/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Finally, we install Jekyll:

```bash
gem install jekyll bundler
```

Notice that we don't use the `root` user :-)

### Running Jekyll Serve

By default, the Jekyll server is launched with the following command (which is the one indicated on your website).

```bash
bundle exec jekyll serve
```

If in the process of building the server there is a dependency problem, for example, there is a missing library to install, it is **necessary to delete** the `Gemfile.lock` file so that it is rebuilt with the installed dependency. This list of dependencies is found in the `Gemfile` file (in Python it would be equivalent to the `requirements.txt` file) and the version of each of the installed gems (packages) is specified. Having a list of dependencies is important for future updates as well as knowing the libraries needed to run the server. Once the `Gemfile.lock` file is deleted, the command shown above is launched again and the dependency errors should end.

### Installing Ruby and Jekyll on MacOS

Follow the Jekyll [page installation guide](https://jekyllrb.com/docs/installation/macos/).


### FAQ

- Error building Jekyll server: 

```bash
jekyll build --incremental --verbose
```
