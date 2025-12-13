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

---

## Build and Run - Complete System

### 1. Start Docker Container

```bash
cd /home/elias/turtlebot4
docker compose up -d
docker exec -it turtlebot4-dev bash
```

### 2. Build All Components

```bash
cd /workspace/workspace
make clean
make

# Build linear controller
cd linear_control
make

# Build joystick
cd ../joystick
make
```

### 3. Start Rosbridge on Robot

**SSH to robot first:**
```bash
ssh ubuntu@192.168.100.100
# password: turtlebot4

# Start rosbridge websocket server
ros2 run rosbridge_server rosbridge_websocket --ros-args -p address:="0.0.0.0" -p port:=9090
```

### 4. Start Bridge (in Docker container)

**Terminal 1:**
```bash
cd /workspace/workspace
./build/turtlebot4_bridge 192.168.100.100
```

You should see:
```
[Rosbridge] Connected to ws://192.168.100.100:9090
[Stats] scan: XXX odom: XXX bumper: XXX
```

### 5. Start Xbox Controller (Joystick)

**Terminal 2:**
```bash
cd /workspace/workspace/joystick
./joy_shm
```

Move left stick to control robot:
- Up/Down: Forward/Backward (linear_x)
- Left/Right: Turn (angular_z)

### 6. Start Linear Controller (Optional)

**Alternative to joystick - drives to waypoints:**
```bash
cd /workspace/workspace/linear_control
./linear_controller
```

### 7. Start Wall Follower (Optional)

**For autonomous navigation:**
```bash
cd /workspace/workspace/wall_follower
./wall_follower
```

### 8. Monitor Shared Memory

**Terminal 3 (optional debug):**
```bash
cd /workspace/workspace
./build/shm_monitor
```

Shows real-time cmd_vel and odom data from shared memory.

---

## GMapping (SLAM)

### Build

```bash
cd /workspace/gmapping
make
```

### Run

**Terminal 1:**
```bash
./bin/core
```

**Terminal 2:**
```bash
./bin/mapping
```

### Output

- `data/data.txt` - Sensor data
- `map.pgm` - Generated map

### View Map

```bash
eog map.pgm
```

---

## Quick Start Summary

**Minimal setup to drive robot with Xbox controller:**

1. Docker → Start bridge
2. Docker → Start joy_shm
3. Move Xbox controller left stick
