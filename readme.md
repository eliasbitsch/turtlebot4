# TurtleBot4 — Project Notes

A compact reference for our TurtleBot4 project: Wi‑Fi credentials, connection info, ROS topics, and team tasks.

---

## Quick Access

- **Wi‑Fi**: `Turtlebot3`
- **Wi‑Fi Password**: `83974764305522315305`
- **TurtleBot4 IP**: `192.168.100.100`
- **SSH**: `ssh ubuntu@192.168.100.100`
- **SSH Password**: `turtlebot4`

## Useful ROS topics

- `/scan` — LIDAR scan
- `/odom` — odometry
- `/cmd_vel` — velocity commands

## Team Tasks

### Philip
- Connect Xbox controller
- Implement gmapping (self-written)

### Ronny
- SLAM

### Vikroriia
- Control bumper
- Linear control

### Elias
- Differential drive controller
- GUI in Python
- Waypoints / autonomous patterns (zig-zag, tornado)

## Nice to have

- Vikroriia: visual detection with YOLO and follower mode
- Max: LLM voice control for robot commands

---

## How to connect quickly

1. Ensure your host is on the same network as `192.168.100.0/24`.
2. SSH into the robot:

    ```bash
    ssh ubuntu@192.168.100.100
    # password: turtlebot4
    ```

3. To monitor topics from your development machine (with ROS set up):

    ```bash
    ros2 topic list
    ros2 topic echo /odom
    ```


    