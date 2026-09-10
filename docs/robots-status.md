# Robot status in Robotics Infrastructure (RI)

## Terrestrial

| Robot | Simulation | Real driver | Sensors | Actuators |
| ------------------- | :-: | :-: | ---------------------------- | ------------------------------------- |
| TurtleBot 2 | ok | ok | camera / stereo | differential drive |
| TurtleBot 3 | ok | | camera, laser, IMU | differential drive |
| Dingo | ok | | laser, IMU | mecanum drive (omnidirectional) |
| Rover 4wd | ok | | laser, IMU | differential drive |
| Vacuum Cleaner | ok | | camera or laser | differential drive |
| Holonomic Logistic | ok | | none | holonomic drive |
| Ackermann Logistic | ok | | none | ackermann steering |
| F1 | ok | | camera or laser | ackermann or holonomic steering |
| Autonomous Car | ok | | camera, lidar or 3x laser | ackermann or holonomic steering |
| MiR100 | ok | | 2x laser, IMU | differential drive |

## Aerial

| Robot | Simulation | Real driver | Sensors | Actuators |
| --------- | :-: | :-: | ------------------ | -------------------- |
| Quadrotor | ok | | camera(s), IMU | multirotor (4 motors), optional magnetic gripper |

## Industrial arms

| Robot | Simulation | Real driver | Sensors | Actuators |
| ------------ | :-: | :-: | ------------------------ | ---------------------------- |
| Ur5 | ok | | camera (optional) | 6-DOF arm + Robotiq 2F-85 gripper |
| Ur3 | ok | | camera (optional) | 6-DOF arm + Robotiq 2F-85 gripper |
| Ur10 Suction | ok | | none | 6-DOF arm + suction gripper |
| Dobot Magician | ok | | none | 4-DOF arm + parallel gripper |
