# TurtleBot4 — Project Notes

A compact reference for the TurtleBot4 project: Wi‑Fi, connection info, ROS topics, team tasks, and quick simulation steps.

## Quick Access

- Wi‑Fi: `Turtlebot3`  
- Wi‑Fi password: `83974764305522315305`  
- TurtleBot4 IP: `192.168.100.100`  
- SSH: `ssh ubuntu@192.168.100.100`  
- SSH password: `turtlebot4`

## Useful ROS topics

- `/scan` — LIDAR scans  
- `/odom` — odometry  
- `/cmd_vel` — velocity commands

## Team Tasks

### Philip
- Connect Xbox controller
- Waypoints in map

### Ronny
- mapping/SLAM
- Map in Rviz

### Viktoriia
- Control bumper  
- Linear control

### Elias
- Websocket connection
- topic parser 
- Valgrind

## Max
- cmd_vel multi-plexer
- Wall follower mit ransac / pca

## Nice to have
- Viktoriia: visual detection with YOLO and follower mode  
- Max: LLM voice control for robot commands

## How to connect quickly

1. Ensure your host is on the same network (192.168.100.0/24).  
2. SSH into the robot:
```bash
ssh ubuntu@192.168.100.100
# password: turtlebot4
```
3. From a machine with ROS 2 installed, monitor topics:
```bash
ros2 topic list
ros2 topic echo /odom
```

## Simulation (local / Docker)

Clone, build and run the simulation:
```bash
git clone https://github.com/eliasbitsch/turtlebot4.git
cd turtlebot4
docker compose up -d

# Wait ~10 minutes for services to become ready
# then enter the dev container:
docker exec -it turtlebot4-dev bash

# Inside the container:
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch turtlebot4_sim turtlebot4_sim.launch.py
```
