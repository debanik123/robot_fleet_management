# Robot Fleet Management System

Welcome to the Robot Fleet Management System repository! This system is designed to help you manage a fleet of robots efficiently. Whether you're deploying robots for industrial tasks, service delivery, or any other purpose, this system provides the tools you need to monitor, control, and optimize your robot fleet.

## Features

- **Dashboard**: View real-time status updates and metrics for all robots in your fleet.
- **Remote Control**: Control individual robots remotely for manual intervention or testing.
- **Task Assignment**: Assign tasks to robots and track their progress.
- **Alerts and Notifications**: Receive alerts and notifications for critical events such as low battery, malfunctions, or task completion.
- **Data Logging and Analytics**: Collect and analyze data to identify trends, optimize performance, and improve efficiency.
- **User Management**: Manage user accounts and access permissions for your team members.

## Getting Started

To get started with the Robot Fleet Management System, follow these steps:

1. **Installation**: Clone this repository to your local machine.
     
   
   ```bash
   sudo apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-turtlebot3-gazebo \
        ros-${ROS_DISTRO}-rosbridge-*

   pip3 install Flask \
         flask-cors
   
   source /opt/ros/${ROS_DISTRO}/setup.bash
   export TURTLEBOT3_MODEL=waffle
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
   git clone https://github.com/debanik123/robot_fleet_management.git
   
   run -->
   
   ros2 run rosbridge_server rosbridge_websocket
   ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
   cd ~/robot_fleet_management/rviz_web
   python3 app.py

# Getting started with rclnode js 

      ```
      sudo apt purge nodejs
      sudo apt autoremove

      sudo apt update
      sudo apt upgrade
      sudo apt install -y curl
      curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
      sudo apt install -y nodejs
      node --version

      git clone https://github.com/RobotWebTools/rclnodejs.git
      cd rclnodejs
      npx rclnodejs-cli -h
      npx rclnodejs-cli generate-ros-messages
      npx rclnodejs-cli create-package -h

      sudo apt-get install ros-humble-ament-*
      sudo apt install ros-humble-test-msgs 
      sudo apt install ros-humble-example-interfaces

      example -->
      npx rclnodejs-cli create-package mypkg --typescript
      cd mypkg
      npm run build
      cd dist
      node index.js

      ```
# run 
``` npm start ```
# kill all node
``` pkill -f node ```
<!-- colcon build -->

<!-- run -->
<!-- source install/setup.bash
ros2 launch mypkg example.launch.py -->



