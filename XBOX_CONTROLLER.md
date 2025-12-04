# Xbox Controller Setup for TurtleBot4

## Quick Start

### 1. Connect Your Xbox Controller

**Wireless (Bluetooth):**
```bash
# Enable Bluetooth and pair controller
bluetoothctl
> scan on
> pair <CONTROLLER_MAC>
> connect <CONTROLLER_MAC>
> trust <CONTROLLER_MAC>
```

**Wired (USB):**
Just plug it in - it should be detected automatically as `/dev/input/js0`

### 2. Verify Controller Detection

```bash
# Check if controller is detected
ls -la /dev/input/js*

# Test controller input
jstest /dev/input/js0

# Or use graphical tool
jstest-gtk
```

### 3. Launch Xbox Controller Teleop

**Inside Docker container:**
```bash
# Terminal 1: Start simulation
ros2 launch turtlebot4_sim turtlebot4_sim.launch.py

# Terminal 2: Start Xbox controller
ros2 launch turtlebot4_sim xbox_teleop.launch.py
```

**On real TurtleBot4 (via SSH):**
```bash
ssh ubuntu@192.168.100.100
ros2 launch turtlebot4_sim xbox_teleop.launch.py
```

## Controller Layout

```
                    Xbox Controller
         
    [LB]                           [RB]
     ↑                              ↑
  Enable                         Turbo
  (Hold)                         (Hold)

    ╔═══╗                       (Y)
    ║ ↑ ║                    (X)   (B)
    ║←+→║                       (A)
    ║ ↓ ║
    ╚═══╝
  Left Stick
  Movement
```

### Button Functions:

- **Left Stick**: 
  - **Up/Down**: Forward/Backward movement
  - **Left/Right**: Turn left/right
  
- **LB (Left Bumper)**: **DEADMAN SWITCH** - Must hold to enable movement
- **RB (Right Bumper)**: **TURBO MODE** - Hold for faster speeds

### Speed Settings:

- **Normal mode** (LB only):
  - Linear: 0.5 m/s max
  - Angular: 1.0 rad/s max

- **Turbo mode** (LB + RB):
  - Linear: 1.0 m/s max
  - Angular: 1.5 rad/s max

## Troubleshooting

### Controller not detected

```bash
# Check USB devices
lsusb | grep -i xbox

# Check input devices
cat /proc/bus/input/devices | grep -A 5 Xbox

# Test with evtest
sudo evtest
```

### Permission issues

```bash
# Add user to input group
sudo usermod -a -G input $USER

# Or create udev rule
echo 'KERNEL=="js[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-joystick.rules
sudo udevadm control --reload-rules
```

### Wrong device path

```bash
# Find correct joystick device
ls -la /dev/input/js*

# Launch with custom device
ros2 launch turtlebot4_sim xbox_teleop.launch.py joy_dev:=/dev/input/js1
```

### Controller inputs not working

```bash
# Test raw joy messages
ros2 topic echo /joy

# Check button/axis values with jstest
jstest /dev/input/js0

# Adjust deadzone if stick is drifting
ros2 param set /joy_node deadzone 0.2
```

## Custom Configuration

Edit `config/xbox_teleop.yaml` to customize:

```yaml
# Change speed limits
scale_linear:
  x: 0.3  # Slower default speed

# Change button mappings
enable_button: 0  # Use A button instead of LB

# Invert axes
inverted_reverse: true  # Invert forward/back
```

## Integration with Existing Code

Your websocket bridge already supports cmd_vel publishing. The Xbox controller will:

1. **Joy node** reads raw controller input → publishes to `/joy` topic
2. **Teleop node** converts `/joy` → publishes to `/cmd_vel`
3. **Your bridge** reads `/cmd_vel` → sends to robot via rosbridge

### With Priority System

If using your PriorityDoubleBuffer for cmd_vel:

- Navigation writes to `write_nav()`
- Xbox teleop writes to `write_teleop()` (auto-activates teleop mode)
- After 500ms without controller input, switch back to navigation

## Advanced: Direct Integration

For lower latency, you can integrate controller directly with your bridge:

```cpp
// In your bridge, read joy messages instead of cmd_vel
client_.subscribe("/joy", [this](auto&, const json& msg) {
    // Parse joy message
    // Convert to cmd_vel
    // Write to priority buffer with TELEOP mode
}, true);
```

## Safety Features

✅ **Deadman switch** - Robot only moves when LB is held  
✅ **Configurable speed limits** - Prevent dangerous speeds  
✅ **Deadzone** - Prevents stick drift from causing movement  
✅ **Priority mode** - Teleop overrides autonomous navigation  

## Testing Checklist

- [ ] Controller detected at `/dev/input/js0`
- [ ] `jstest` shows button/axis changes
- [ ] `/joy` topic publishes messages when buttons pressed
- [ ] `/cmd_vel` publishes when LB held + stick moved
- [ ] Robot moves forward when stick pushed up (with LB held)
- [ ] Robot stops when LB released
- [ ] Turbo mode increases speed when RB held

## Resources

- [ROS 2 Joy Package](https://index.ros.org/p/joy/)
- [Teleop Twist Joy](https://index.ros.org/p/teleop_twist_joy/)
- [Xbox Controller on Linux](https://wiki.archlinux.org/title/Gamepad)
