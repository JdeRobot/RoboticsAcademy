# Info for developers: exercise status about direct ROS2 support, C++ support and which simulator it is based on

## 🟢 Operational Exercises

| Exercise ID                          | Python simpleAPI | Python ROS2 | C++ simpleAPI | C++ ROS2 | URL Docs                          | Video                        | 🟧 Gazebo Harmonic |          Notes          |
| ------------------------------------ | :--------------: | :---------: | :-----------: | :------: | --------------------------------- | ---------------------------- | :----------------: | :---------------------: |
| Basic Vacuum Cleaner                 |        ok        |     ok      |      ok       |    ok    | [Docs][doc_basic_vacuum_cleaner]  | [Link][vid_basic_vacuum]     |         ok         |           FSM           |
| Follow Line                          |        ok        |     ok      |      ok       |    ok    | [Docs][doc_follow_line]           | [Link][vid_follow_line]      |         ok         |       PID control       |
| Obstacle Avoidance                   |        ok        |     ok      |      ok       |    ok    | [Docs][doc_obstacle_avoidance]    | [Link][vid_obstacle]         |         ok         |           VFF           |
| Global navigation                    |        ok        |     ok      |      ok       |    ok    | [Docs][doc_global_navigation]     | [Link][vid_global_nav]       |         ok         |           GPP           |
| Laser Gridmap mapping                |        ok        |     ok      |      ok       |    ok    | [Docs][doc_laser_grid_map]        | [Link][vid_laser_mapping]    |         ok         |                         |
| MonteCarlo Laser Loc                 |        ok        |     ok      |      ok       |    ok    | [Docs][doc_mntcrl_laser_loc]      | [Link][vid_montecarlo_laser] |         ok         |          AMCL           |
| MonteCarlo Visual Loc                |        ok        |     ok      |      ok       |    ok    | [Docs][doc_mntcrl_visual_loc]     | -                            |         ok         |          AMCL           |
| Marker-based visual Loc              |        ok        |     ok      |      ok       |    ok    | [Docs][doc_marker_visual_loc]     | [Link][vid_marker_loc]       |         ok         |           PnP           |
| Localized Vacuum Cleaner             |        ok        |     ok      |      ok       |    ok    | [Docs][doc_localized_vacuum]      | [Link][vid_localized_vacuum] |         ok         |           BSA           |
| Autoparking                          |        ok        |     ok      |      ok       |    ok    | [Docs][doc_autoparking]           | [Link][vid_autoparking]      |         ok         |                         |
| Amazon Warehouse                     |        ok        |     ok      |      ok       |    ok    | [Docs][doc_amazon_warehouse]      | [Link][vid_amazon_warehouse] |         ok         |                         |
| Follow Person                        |        ok        |     ok      |      ok       |    ok    | [Docs][doc_follow_person]         | [Link][vid_follow_person]    |         ok         |                         |
| Drone Follow Road                    |        ok        |     ok      |      ok       |    ok    | [Docs][doc_drone_follow_road]     | -                            |         ok         |                         |
| Drone Rescue people                  |        ok        |     ok      |      ok       |    ok    | [Docs][doc_drone_rescue_people]   | -                            |         ok         |                         |
| Drone Gymkhana                       |        ok        |     ok      |      ok       |    ok    | [Docs][doc_drone_gymkhana]        | [Link][vid_drone_gymkhana]   |         ok         |                         |
| Drone Power tower Inspection         |        ok        |     ok      |      ok       |    ok    | [Docs][doc_drone_inspection]      | -                            |         ok         |                         |
| Visual 3D reconstruction             |        ok        |     ok      |      ok       |    ok    | [Docs][doc_3d_reconstruction]     | [Link][vid_3d_reconstr]      |         ok         |                         |
| DL e2e Visual control                |        ok        |     ok      |      ok       |    ok    | [Docs][doc_dl_e2e_visual_control] | -                            |         ok         |                         |
| Basic Computer Vision                |        ok        |     ok      |      ok       |    ok    | [Docs][doc_basic_comp_vision]     | [Link][vid_basic_cv]         |        none        |                         |
| DL image classification              |        ok        |     ok      |      ok       |    ok    | [Docs][doc_dl_classification]     | -                            |        none        |                         |
| DL visual object detection           |        ok        |     ok      |      ok       |    ok    | [Docs][doc_visual_obj_detection]  | -                            |        none        |                         |
| Pick and Place                       |        ok        |     ok      |      ok       |    ok    | [Docs][doc_pick_and_place]        | [Link][vid_pick_place]       |         ok         |      reset failure      |
| Machine Vision with industrial robot |        ok        |     ok      |      ok       |    ok    | [Docs][doc_machine_vision_indus]  | [Link][vid_machine_vision]   |         ok         |      reset failure      |
| Car junction                         |        ok        |     ok      |      ok       |    ok    | [Docs][doc_car_junction]          | -                            |         ok         |                         |
| Drone Labyrinth escape               |        ok        |     ok      |      ok       |    ok    | -                                 | -                            |         ok         |                         |
| Drone Position control               |        ok        |     ok      |      ok       |    ok    | -                                 | -                            |         ok         |                         |
| Drone Follow TurtleBot               |        ok        |     ok      |      ok       |    ok    | -                                 | -                            |         ok         |                         |
| Drone Package delivery               |        ok        |     ok      |      ok       |    ok    | -                                 | -                            |         ok         |      reset failure      |
| Drone hangar                         |        ok        |     ok      |      ok       |    ok    | -                                 | -                            |         ok         |                         |
| Drone Visual Lander                  |        ok        |             |               |          | -                                 | -                            |         ok         | missing actor collision |
| Drone Cat and Mouse                  |        ok        |             |               |          | -                                 | -                            |         ok         |                         |
|                                      |                  |             |               |          |                                   |                              |                    |                         |
| Dynamic Window Approach              |        ok        |     ok      |      ok       |    ok    | -                                 | [Link][vid_dynamic_window]   |         ok         |                         |
| Visual Odometry                      |        ok        |     ok      |      ok       |    ok    | -                                 | [Link][vid_visual_odometry]  |         ok         |                         |
| Visibility Graph Navigation          |        ok        |     ok      |      ok       |    ok    | -                                 | -                            |         ok         |                         |
| Rapidly Exploring Random Trees nav   |        ok        |     ok      |      ok       |    ok    | -                                 | -                            |         ok         |                         |
| Line-based Mapping                   |        ok        |     ok      |      ok       |    ok    | -                                 | -                            |         ok         |                         |

## 🚧 Exercises under repair / workshop state

| Exercise ID              | Python simpleAPI | Python ROS2 | C++ simpleAPI | C++ ROS2 | Video | 🟧 Gazebo Harmonic | Notes  |
| ------------------------ | :--------------: | :---------: | :-----------: | :------: | ----- | :----------------: | ------ |
| Digital Image Processing |        ok        |             |               |          | -     |        none        | broken |

[vid_follow_line]: https://www.youtube.com/watch?v=HRZC1-tGW-s
[vid_obstacle]: https://www.youtube.com/watch?v=6JvlBnJsP90
[vid_autoparking]: https://www.youtube.com/watch?v=2tDKgsM8nyA
[vid_follow_person]: https://www.youtube.com/watch?v=2E5op15e56Q
[vid_basic_cv]: https://www.youtube.com/watch?v=vXg-QYPyvhk
[vid_machine_vision]: https://www.youtube.com/watch?v=ELx35ymlQXk
[vid_basic_vacuum]: https://www.youtube.com/watch?v=ceQO6Z_ZiCM
[vid_pick_place]: https://www.youtube.com/watch?v=y41iRZfnB7E
[vid_amazon_warehouse]: https://www.youtube.com/watch?v=t-sJ1vK8yGk
[vid_laser_mapping]: https://www.youtube.com/watch?v=rH7A-gPXplU
[vid_localized_vacuum]: https://www.youtube.com/watch?v=cajn0qb-oeY
[vid_global_nav]: https://www.youtube.com/watch?v=mOqtXiUAny0
[vid_marker_loc]: https://www.youtube.com/watch?v=HF57L6ekrRg
[vid_drone_gymkhana]: https://www.youtube.com/watch?v=uNVWs_-JfV8
[vid_montecarlo_laser]: https://www.youtube.com/watch?v=mkEZ-ffX6cA
[vid_3d_reconstr]: https://www.youtube.com/watch?v=nJeppI5i5H0
[vid_dynamic_window]: https://www.youtube.com/watch?v=0fsE49EijDc
[vid_visual_odometry]: https://www.youtube.com/watch?v=B95ZKyl_bIk
[doc_basic_vacuum_cleaner]: https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner
[doc_follow_line]: https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line
[doc_obstacle_avoidance]: https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/obstacle_avoidance
[doc_global_navigation]: https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation
[doc_laser_grid_map]: https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/laser_mapping
[doc_mntcrl_laser_loc]: https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/montecarlo_laser_loc
[doc_mntcrl_visual_loc]: https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/montecarlo_visual_loc
[doc_marker_visual_loc]: https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/marker_visual_loc
[doc_localized_vacuum]: https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner_loc
[doc_autoparking]: https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/autoparking
[doc_amazon_warehouse]: https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/amazon_warehouse
[doc_follow_person]: https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/follow_person
[doc_drone_follow_road]: https://jderobot.github.io/RoboticsAcademy/exercises/Drones/follow_road
[doc_drone_rescue_people]: https://jderobot.github.io/RoboticsAcademy/exercises/Drones/rescue_people
[doc_drone_gymkhana]: https://jderobot.github.io/RoboticsAcademy/exercises/Drones/drone_gymkhana
[doc_drone_inspection]: https://jderobot.github.io/RoboticsAcademy/exercises/Drones/power_tower_inspection
[doc_3d_reconstruction]: https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/3d_reconstruction
[doc_dl_e2e_visual_control]: https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/end_to_end_visual_control
[doc_basic_comp_vision]: https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/basic_computer_vision
[doc_dl_classification]: https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/image_classification
[doc_visual_obj_detection]: https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/object_detection
[doc_pick_and_place]: https://jderobot.github.io/RoboticsAcademy/exercises/IndustrialRobots/pick_place
[doc_machine_vision_indus]: https://jderobot.github.io/RoboticsAcademy/exercises/IndustrialRobots/machine_vision
[doc_car_junction]: https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/car_junction
