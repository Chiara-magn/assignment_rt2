# assignment_rt2
Repository for assignment2 Research Track 1


# **Research Track I – Assignment 2**

This project implements a multi‑node ROS2 system for robot teleoperation and safety monitoring in a Gazebo simulation.  
The system allows the user to drive the robot, detects obstacles, triggers a rollback procedure, publishes custom obstacle information, and allows the user tu choose obstacles threshold and velocity statistics.

---

## **Node Description**

### **1. User Controller (node 1)**  
This node provides a keyboard‑based interface to control the robot.

The user can:
- Move the robot using **W/S/A/D**
- Stop the robot (**X**)
- Change the safety threshold via service (**T**)
- Change linear and angular velocities (**V**)
- Request the average of the last 5 velocity commands (**M**)

Teleoperation is automatically disabled when the safety system is active.  
Commands are published to `/user_cmd` at 20 Hz.

---

### **2. Safety Detector (node 2)**  
This node processes LaserScan data to detect obstacles.

The node:
- Computes the **minimum distance**
- Determines the **direction** of the closest obstacle (left, front, right)
- Publishes a custom `ObstacleInfo` message
- Activates a **safety lock** when the robot is too close to an obstacle
- Provides a service to update the threshold

---

### **3. Rollback Controller (node 3)**  
This node generates a backward motion when safety is active.

The node:
- Subscribes to `/safety_lock`
- Publishes a backward velocity on `/rollback_cmd`
- Stops when safety is released

---

### **4. Command Multiplexer (node 4)**  
This node selects which velocity command is sent to the robot.

The node:
- Gives priority to rollback commands when safety is active
- Otherwise forwards teleop commands
- Publishes the final command to `/cmd_vel`
- Stores the last 5 velocity commands
- Provides a service to compute the **average linear and angular velocity**

---

## **Custom Message**

### `ObstacleInfo.msg`
```
bool is_obstacle
float32 min_distance
string direction
float32 threshold
```

---

## **Services**

### `SetThreshold.srv`
Updates the safety threshold.

### `GetAverages.srv`
Returns the average linear and angular velocity of the last 5 commands.

---

## **Launch File**

A dedicated launch file is provided to automatically start the core nodes:

- `safety_detector`
- `rollback_controller`
- `cmd_mux`

### **Run the system launch**
```bash
ros2 launch assignment2_rt system.launch.py
```

This launch file does **not** start the User Controller, since it requires keyboard input and must be run manually.

---

## **How to Run the Full System**

### **1. Build the workspace**
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select assignment2_rt
source install/setup.bash
```

### **2. Start the simulation**
```bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```

### **3. Start the core nodes**
```bash
ros2 launch assignment2_rt system.launch.py
```

### **4. Start the User Controller**
```bash
ros2 run assignment2_rt user_controller
```

---

## **User controller Commands**

| Key | Action |
|-----|--------|
| **w** | Forward |
| **s** | Backward |
| **a** | Rotate left |
| **d** | Rotate right |
| **x** | Stop |
| **t** | Set new threshold |
| **v** | Set linear/angular velocity |
| **m** | Get averages of last 5 velocities |
| **q** | Quit |

---



