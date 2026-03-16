# Info for developers: exercise status about direct ROS2 support, C++ support and which simulator it is based on
<br/>

| Exercise ID                           | Python simpleAPI | Python ROS2 | C++ simpleAPI | C++ ROS2 | Video | Simulator                 | Notes       |
| ------------------------------------- | :--------------: | :---------: | :-----------: | :------: | :---: | ------------------------- | ----------- |
| Basic Vacuum Cleaner                  | ok               | ok          | ok            | ok       | -     | Gazebo Harmonic + Classic | FSM         |
| Follow Line                           | ok               | ok          | ok            | ok       | [Link][vid_follow_line] | Gazebo Harmonic + Classic | PID control |
| Obstacle Avoidance                    | ok               |             |               |          | [Link][vid_obstacle] | Gazebo Harmonic + Classic | VFF         |
| Global navigation                     | ok               |             |               |          | -     | Gazebo Harmonic + Classic | GPP         |
| Laser Gridmap mapping                 | ok               |             |               |          | -     | Gazebo Harmonic           |             |
| MonteCarlo Laser Loc                  | ok               |             |               |          | -     | Gazebo Harmonic + Classic | AMCL        |
| MonteCarlo Visual Loc                 | ok               |             |               |          | -     | Gazebo Harmonic + Classic | AMCL        |
| Marker-based visual Loc               | ok               |             |               |          | -     | Gazebo Harmonic           | PnP         |
| Localized Vacuum Cleaner              | ok               |             |               |          | -     | Gazebo Harmonic + Classic | BSA         |
| Autoparking                           | ok               |             |               |          | [Link][vid_autoparking] | Gazebo Harmonic + Classic |             |
| Amazon Warehouse                      | ok               |             |               |          | -     | Gazebo Harmonic + Classic                 |             |
| Follow Person                         | ok               |             |               |          | [Link][vid_follow_person] | Gazebo 11                 |             |
| Drone Follow Road                     | ok               |             |               |          | -     | Gazebo Harmonic           |             |
| Drone Rescue people                   | ok               |             |               |          | -     | Gazebo Harmonic           |             |
| Drone Gymkhana                        | ok               |             |               |          | -     | Gazebo Harmonic           |             |
| Drone Power tower Inspection          | ok               |             |               |          | -     | Gazebo Harmonic           |             |
| Visual 3D reconstruction              | ok               |             |               |          | -     | Gazebo Harmonic + Classic |             |
| DL e2e Visual control                 | ok               |             |               |          | -     | Gazebo Harmonic + Classic |             |
| Basic Computer Vision                 | ok               |             |               |          | [Link][vid_basic_cv] | none                      |             |
| DL image classification               | ok               |             |               |          | -     | none                      |             |
| DL visual object detection            | ok               |             |               |          | -     | none                      |             |
| Pick and Place                        | ok               |             |               |          | -     | Gazebo Harmonic + Classic                  |             |
| Machine Vision with industrial robot  | ok               |             |               |          | [Link][vid_machine_vision] | Gazebo 11                 |             |
| Car junction                          | ok               |             |               |          | -     | Gazebo Harmonic           |             |
| Digital Image Processing              | ok               |             |               |          | -     | none                      | broken      |
| Drone Labyrinth escape                | ok               |             |               |          | -     | Gazebo Harmonic           |             |
| Drone Position control                |                  |             |               |          | -     |                           | WIP         |
| Drone Follow TurtleBot                |                  |             |               |          | -     |                           | broken      |
| Drone Package delivery                |                  |             |               |          | -     |                           | WIP         |
| Drone hangar                          |                  |             |               |          | -     |                           | broken      |
| Drone Visual Lander                   |                  |             |               |          | -     |                           | broken      |
| Drone Cat and Mouse                   |                  |             |               |          | -     |                           | broken      |

[vid_follow_line]: https://www.youtube.com/watch?v=HRZC1-tGW-s
[vid_obstacle]: https://www.youtube.com/watch?v=6JvlBnJsP90
[vid_autoparking]: https://www.youtube.com/watch?v=2tDKgsM8nyA
[vid_follow_person]: https://www.youtube.com/watch?v=2E5op15e56Q
[vid_basic_cv]: https://www.youtube.com/watch?v=vXg-QYPyvhk
[vid_machine_vision]: https://www.youtube.com/watch?v=ELx35ymlQXk
