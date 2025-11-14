ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=maze

ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: -0.2}, angular: {z: 0.0}}}" -r 10