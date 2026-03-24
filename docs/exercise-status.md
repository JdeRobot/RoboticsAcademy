# Info for developers: exercise status about direct ROS2 support, C++ support and which simulator it is based on

## 🟢 Operational Exercises

| Exercise ID                          | Python simpleAPI | Python ROS2 | C++ simpleAPI | C++ ROS2 | Video                    | 🟨 Gazebo Classic | 🟧 Gazebo Harmonic | Notes       |
| ------------------------------------ | :--------------: | :---------: | :-----------: | :------: | ------------------------ | :---------------: | :----------------: | ----------- |
| Basic Vacuum Cleaner                 |        ok        |     ok      |      ok       |    ok    | [Link][vid_basic_vacuum] |        ok         |         ok         | FSM         |
| Follow Line                          |        ok        |     ok      |      ok       |    ok    | [Link][vid_follow_line]  |        ok         |         ok         | PID control |
| Obstacle Avoidance                   |        ok        |     ok      |               |          | [Link][vid_obstacle]     |        ok         |         ok         | VFF         |
| Global navigation                    |        ok        |     ok      |               |          | -                        |        ok         |         ok         | GPP         |
| Laser Gridmap mapping                |        ok        |             |               |          | -                        |                   |         ok         |             |
| MonteCarlo Laser Loc                 |        ok        |             |               |          | -                        |        ok         |         ok         | AMCL        |
| MonteCarlo Visual Loc                |        ok        |             |               |          | -                        |        ok         |         ok         | AMCL        |
| Marker-based visual Loc              |        ok        |             |               |          | -                        |                   |         ok         | PnP         |
| Localized Vacuum Cleaner             |        ok        |     ok      |               |          | -                        |        ok         |         ok         | BSA         |
| Autoparking                          |        ok        |             |               |          | [Link][vid_autoparking]  |        ok         |         ok         |             |
| Amazon Warehouse                     |        ok        |             |               |          | -                        |        ok         |         ok         |             |
| Follow Person                        |        ok        |             |               |          | [Link][vid_follow_person]|        ok         |                    |             |
| Drone Follow Road                    |        ok        |             |               |          | -                        |                   |         ok         |             |
| Drone Rescue people                  |        ok        |             |               |          | -                        |                   |         ok         |             |
| Drone Gymkhana                       |        ok        |             |               |          | -                        |                   |         ok         |             |
| Drone Power tower Inspection         |        ok        |             |               |          | -                        |                   |         ok         |             |
| Visual 3D reconstruction             |        ok        |             |               |          | -                        |        ok         |         ok         |             |
| DL e2e Visual control                |        ok        |             |               |          | -                        |        ok         |         ok         |             |
| Basic Computer Vision                |        ok        |     ok      |               |          | [Link][vid_basic_cv]     |       none        |        none        |             |
| DL image classification              |        ok        |     ok      |               |          | -                        |       none        |        none        |             |
| DL visual object detection           |        ok        |     ok      |               |          | -                        |       none        |        none        |             |
| Pick and Place                       |        ok        |             |               |          | [Link][vid_pick_place]   |        ok         |         ok         |             |
| Machine Vision with industrial robot |        ok        |             |               |          | [Link][vid_machine_vision]|       ok         |                    |             |
| Car junction                         |        ok        |             |               |          | -                        |                   |         ok         |             |

## 🚧 Exercises under repair / workshop state

| Exercise ID                          | Python simpleAPI | Python ROS2 | C++ simpleAPI | C++ ROS2 | Video | 🟨 Gazebo Classic | 🟧 Gazebo Harmonic | Notes  |
| ------------------------------------ | :--------------: | :---------: | :-----------: | :------: | ----- | :---------------: | :----------------: | ------ |
| Digital Image Processing             |        ok        |             |               |          | -     |       none        |        none        | broken |
| Drone Labyrinth escape               |        ok        |             |               |          | -     |                   |         ok         |        |
| Drone Position control               |                  |             |               |          | -     |                   |                    | WIP    |
| Drone Follow TurtleBot               |                  |             |               |          | -     |                   |                    | broken |
| Drone Package delivery               |                  |             |               |          | -     |                   |                    | WIP    |
| Drone hangar                         |                  |             |               |          | -     |                   |                    | broken |
| Drone Visual Lander                  |                  |             |               |          | -     |                   |                    | broken |
| Drone Cat and Mouse                  |                  |             |               |          | -     |                   |                    | broken |

[vid_follow_line]: https://www.youtube.com/watch?v=HRZC1-tGW-s
[vid_obstacle]: https://www.youtube.com/watch?v=6JvlBnJsP90
[vid_autoparking]: https://www.youtube.com/watch?v=2tDKgsM8nyA
[vid_follow_person]: https://www.youtube.com/watch?v=2E5op15e56Q
[vid_basic_cv]: https://www.youtube.com/watch?v=vXg-QYPyvhk
[vid_machine_vision]: https://www.youtube.com/watch?v=ELx35ymlQXk
[vid_basic_vacuum]: https://www.youtube.com/watch?v=ceQO6Z_ZiCM
[vid_pick_place]: https://www.youtube.com/watch?v=y41iRZfnB7E
