# 🤖 Cafe Butler Robot

An autonomous food delivery robot built with ROS2 for the French Door Cafe. The robot replaces human waitstaff by navigating between the kitchen and customer tables to deliver food orders.

---

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [How to Run](#how-to-run)
- [Testing Milestones](#testing-milestones)
- [Project Structure](#project-structure)
- [ROS2 Topics](#ros2-topics)
- [Waypoints](#waypoints)

---

## 🏪 Overview

The robot operates in a cafe with 3 tables and 1 kitchen. Its workflow:

```
HOME → KITCHEN → TABLE (1, 2, or 3) → HOME
```

Built using:
- **ROS2 Humble** - Robot Operating System
- **Nav2** - Autonomous navigation
- **SLAM Toolbox** - Map generation
- **Gazebo** - 3D simulation
- **TurtleBot3 Burger** - Robot model

---

## ✅ Features (7 Milestones)

| # | Milestone | Description |
|---|-----------|-------------|
| 1 | Basic Delivery | Home → Kitchen → Table → Home |
| 2 | Timeout Handling | Robot returns home if no confirmation received |
| 3a | Kitchen Timeout | No kitchen confirm → go home |
| 3b | Table Timeout | No table confirm → go kitchen → go home |
| 4 | Order Cancellation | Smart return path based on cancellation point |
| 5 | Multiple Orders | Deliver to all tables then return home |
| 6 | Skip Unconfirmed | Skip table if no confirm, go kitchen at end |
| 7 | Cancel Specific Table | Skip canceled table, deliver to remaining |

---

## 🛠 Prerequisites

```bash
# ROS2 Humble on Ubuntu 22.04
sudo apt install ros-humble-turtlebot3 \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-rviz2
```

---

## 📦 Installation

```bash
# Clone the repository
git clone <your-repo-url>
cd goat_robot_ws

# Source ROS2
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Add to bashrc permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# Build
colcon build
source install/setup.bash
```

---

## 🚀 How to Run

### Step 1: Launch Simulation
```bash
# Terminal 1
source ~/goat_robot_ws/install/setup.bash
ros2 launch cafe_butler_robot cafe.launch.py
```

### Step 2: Launch RViz2 with Nav2
```bash
# Terminal 2
ros2 launch nav2_bringup rviz_launch.py
```
> In RViz2: Click **2D Pose Estimate** and click on the map to set robot's initial position

### Step 3: Run Robot Controller
```bash
# Terminal 3
source ~/goat_robot_ws/install/setup.bash
ros2 run cafe_butler_robot robot_controller
```

### Step 4: Send Orders
```bash
# Send order for table1
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table1'}"

# Confirm kitchen pickup
ros2 topic pub --once /kitchen_confirm std_msgs/msg/Bool "{data: true}"

# Confirm table delivery
ros2 topic pub --once /table_confirm std_msgs/msg/Bool "{data: true}"
```

---

## 🧪 Testing Milestones

### Milestone 1 - Basic Delivery
```bash
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table1'}"
ros2 topic pub --once /kitchen_confirm std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /table_confirm std_msgs/msg/Bool "{data: true}"
```

### Milestone 2 - Timeout (just wait 8 seconds, no confirmation needed)
```bash
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table1'}"
# Don't send any confirmation - robot will return home after 8 seconds
```

### Milestone 4 - Cancel Order
```bash
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table1'}"
ros2 topic pub --once /cancel_order std_msgs/msg/String "{data: 'table1'}"
```

### Milestone 5 - Multiple Orders
```bash
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table1'}"
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table2'}"
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table3'}"
ros2 topic pub --once /kitchen_confirm std_msgs/msg/Bool "{data: true}"
# Confirm each table as robot arrives
ros2 topic pub --once /table_confirm std_msgs/msg/Bool "{data: true}"
```

### Milestone 7 - Cancel Specific Table
```bash
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table1'}"
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table2'}"
ros2 topic pub --once /dispatch_order std_msgs/msg/String "{data: 'table3'}"
ros2 topic pub --once /cancel_order std_msgs/msg/String "{data: 'table2'}"
# Robot delivers to table1 and table3, skips table2
```

---

## 📁 Project Structure

```
cafe_butler_robot/
├── cafe_butler_robot/
│   ├── robot_controller.py      # Main brain - state machine
│   ├── order_manager.py         # Order queue management
│   ├── confirmation_node.py     # Confirmation handling
│   └── __init__.py
├── launch/
│   └── cafe.launch.py           # Main launch file
├── worlds/
│   └── cafe_world.world         # Gazebo cafe world
├── maps/
│   ├── cafe_map.pgm             # Map image
│   └── cafe_map.yaml            # Map metadata
├── config/
│   └── nav2_params.yaml         # Nav2 parameters
├── package.xml
├── setup.py
└── README.md
```

---

## 📡 ROS2 Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/dispatch_order` | std_msgs/String | Subscribe | Receive table order |
| `/cancel_order` | std_msgs/String | Subscribe | Cancel order |
| `/kitchen_confirm` | std_msgs/Bool | Subscribe | Kitchen confirmation |
| `/table_confirm` | std_msgs/Bool | Subscribe | Table confirmation |
| `/navigate_to_pose` | nav2_msgs/Action | Client | Send navigation goal |

---

## 📍 Waypoints

| Location | X | Y | Description |
|----------|---|---|-------------|
| home | 0.0 | -1.0 | Robot starting position |
| kitchen | 1.5 | 0.0 | Food pickup |
| table1 | 2.0 | 2.0 | Customer table 1 |
| table2 | -2.0 | 2.0 | Customer table 2 |
| table3 | 0.0 | -2.0 | Customer table 3 |

---

## 🗺 Remapping (SLAM)

To create a new map:
```bash
# Terminal 1 - Launch Gazebo
ros2 launch cafe_butler_robot cafe.launch.py

# Terminal 2 - Start SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3 - Drive robot
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 4 - Save map when done
ros2 run nav2_map_server map_saver_cli \
  -f ~/goat_robot_ws/src/cafe_butler_robot/maps/cafe_map
```

---

## 📄 License
Apache License 2.0

---

*Built for GOAT Robotics ROS Developer Assessment*
