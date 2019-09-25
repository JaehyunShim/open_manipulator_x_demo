# TurtleBot3 Demo

## Usage

### Install ROS 2 Dashing
```sh
$ sudo apt-get update && sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/rjshim/open_manipulator_x_demo/master/install_ros_dashing.sh && chmod 755 ./install_ros_dashing.sh && bash ./install_ros_dashing.sh
```

### Install ROS packages and Build
```sh
(Move to colcon workspace)
$ cd ~/robotis_ws/src/

(Download packages)
$ git clone https://github.com/rjshim/cmake_modules.git -b ros2-devel
$ git clone https://github.com/rjshim/joint_state_publisher.git -b ros2-devel
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b ros2
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git -b ros2
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git -b ros2
$ git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git -b ros2
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git -b ros2
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator.git -b ros2
$ git clone https://github.com/rjshim/open_manipulator_x_demo.git
$ cd ~/robotis_ws && colcon build --symlink-install

(Build)
$ cd ~/catkin_ws && catkin_make
```

### Execute ROS packages
- Run below in the terminal window.

```sh
(Run Controller)
$ ros2 run open_manipulator_x_controller create_udev_rules
$ ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

(Run Demo)
$ ros2 launch open_manipulator_x_demo open_manipulator_x_demo.py
```

### Reference
- [OpenMANIPUALTOR repository](https://github.com/ROBOTIS-GIT/open_manipulator/network)
- [OpenMANIPUALTOR eManual](http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros2_setup/#ros-setup)
