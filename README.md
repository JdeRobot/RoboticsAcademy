# JdeRobot Academy: Learn Robotics and Computer Vision 


JdeRobot Academy is an **open source** collection of exercises to learn robotics in a practical way. Gazebo simulator is the main tool required, as ROS. Its latest documentation (including installation recipes, current available exercises and illustrative videos) is on its <a href="https://jderobot.github.io/RoboticsAcademy">webpage</a>.

If you are a contributor, please note that we use GitHub Pages and a Jekyll theme (MinimalMistakes) for Academy web page. Feel free to install locally Jekyll so you can test your changes before submitting your pull-request. Here you are the directions for it:


# Jekyll Local Installation

## Prerequisites

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

## Running Jekyll Serve

By default, the Jekyll server is launched with the following command (which is the one indicated on your website).

```bash
bundle exec jekyll serve
```

If in the process of building the server there is a dependency problem, for example, there is a missing library to install, it is **necessary to delete** the `Gemfile.lock` file so that it is rebuilt with the installed dependency. This list of dependencies is found in the `Gemfile` file (in Python it would be equivalent to the `requirements.txt` file) and the version of each of the installed gems (packages) is specified. Having a list of dependencies is important for future updates as well as knowing the libraries needed to run the server. Once the `Gemfile.lock` file is deleted, the command shown above is launched again and the dependency errors should end.

## Notes for exercise cards.

- Teaser Images size: multiple of 600x400px


## FAQ

- Error building Jekyll server: 

```bash
jekyll build --incremental --verbose
```

# Development status of the exercises

Available exercises and its status:

|      Category       |        Exercise         |     Status     |      Robot       | Infrastructure | Solution |  Theory  | Language |
| :-----------------: | :---------------------: | :------------: | :--------------: | :------------: | :------: | :------: | :------: |
|                     | 3D Reconstruction       | **Production** |      Camera      |    **DONE**    | **DONE** | **DONE** |  Python  |
|       Vision        | Color Filter            | **Production** |      Camera      |    **DONE**    | **DONE** | **DONE** |  Python  |
|                     | FollowFace              | **Production** |  Sony Evi d100p  |    **DONE**    | **DONE** | **DONE** |  Python  |
| :-----------------: | :---------------------: | :------------: | :--------------: | :------------: | :------: | :------: | :------: |
|                     | Bump&Go                 | **Production** | TurtleBot        | **DONE**       | **DONE** | **DONE** |  Python  |
|                     | Amazon Warehouse        | Prototype      | AmazonBot        | **DONE**       | WIP      |  Review  |  Python  |
|   Mobile Robots     | Follow Line Turtlebot   | **Production** | TurtleBot        | **DONE**       | ?        | **DONE** |  Python  |
|                     | Laser Loc               | **Production** | Roomba           | **DONE**       | **DONE** | **DONE** |  Python  |
|                     | Vacuum Cleaner          | **Production** | Roomba           | **DONE**       | **DONE** | **DONE** |  Python  |
|                     | Vacuum SLAM             | **Production** | Roomba           | **DONE**       | **DONE** | **DONE** |  Python  |
| :-----------------: | :---------------------: | :------------: | :--------------: | :------------: | :------: | :------: | :------: |
|                     | Autopark                | **Production** | TaxiHolo         | **DONE**       | **DONE** | **DONE** |  Python  |
|                     | Cross Roads with Stop   | **Production** | Opel             | **DONE**       | **DONE** | **DONE** |  Python  |
|   Autonomous Cars   | Qualifying F1           | WIP            | F1               | WIP            | WIP      | Review   |  Python  |
|                     | Global Navigation (GPP) | **Production** | TaxiHolo         | **DONE**       | **DONE** | **DONE** |  Python  |
|                     | Obstacle Avoidance (VFF)| **Production** | F1               | **DONE**       | **DONE** | **DONE** |  Python  |
|                     | Follow Line             | **Production** | F1               | **DONE**       | **DONE** | **DONE** |  Python  |
| :-----------------: | :---------------------: | :------------: | :--------------: | :------------: | :------: | :------: | :------: |
|                     | Drone Hangar            | Prototype      | Drone            | WIP            | WIP      | **DONE** |  Python  |
|                     | Follow Road             | **Production** | Drone            | **DONE**       | ?        | **DONE** |  Python  |
|                     | Follow Turtlebot        | **Production** | Drone            | **DONE**       | ?        | **DONE** |  Python  |
|                     | Drone Cat Mouse         | **Production** | Drone            | **DONE**       | **DONE** | **DONE** |  Python  |
|       Drones        | Labyrinth Escape        | **Production** | Drone            | **DONE**       | ?        | **DONE** |  Python  |
|                     | Position Control        | **Production** | Drone            | **DONE**       | **DONE** | **DONE** |  Python  |
|                     | Rescue People           | **Production** | Drone            | **DONE**       | Review   | **DONE** |  Python  |
|                     | Visual Lander           | ?              | Drone            | **DONE**       | ?        | Review   |  Python  |
|                     | Path Planning and Navigation in 3D | **Prototype** | Drone  | **DONE**       | ?        | ?        |  Python  |
| :-----------------: | :---------------------: | :------------: | :--------------: | :------------: | :------: | :------: | :------: |
|  Industrial Robots  | Look and Push           | ?              | Robotic Arm      | ?              | ?        | ?        |  Python  |








