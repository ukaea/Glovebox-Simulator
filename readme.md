# ros_kortex

## Installation

### Setup

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

This package has been tested under ROS Kinetic (Ubuntu 16.04) and ROS Melodic (Ubuntu 18.04).
You can find the instructions to install ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and ROS Melodic [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

[Google Protocol Buffers](https://developers.google.com/protocol-buffers/) is used by Kinova to define the Kortex APIs and to automatically generate ROS messages, services and C++ classes from the Kortex API `.proto` files. The installation of Google Protocol Buffers is required by developers implementing new APIs with the robot. However, since we already provide all the necessary generated files on GitHub, this is not required for most end users of the robot.

### Build

These are the instructions to run in a terminal to create the workspace, clone the `ros_kortex` repository, install the necessary ROS dependencies and build the package:

        sudo apt install python3 python3-pip
        sudo python3 -m pip install conan
        conan config set general.revisions_enabled=1
        conan profile new default --detect > /dev/null
        conan profile update settings.compiler.libcxx=libstdc++11 default
        mkdir -p catkin_workspace/src
        cd catkin_workspace/src
        git clone https://github.com/Kinovarobotics/ros_kortex.git
        cd ../
        rosdep install --from-paths src --ignore-src -y
        catkin_make
        source devel/setup.bash

As you see, there are instructions to install the Conan package manager. You can learn more about why we use Conan [in this specific section of the kortex_driver readme](kortex_driver/readme.md#conan).

## Usage

The [spawn_kortex_robot.launch file](launch/spawn_kortex_robot.launch) launches the arm simulation in [Gazebo](http://gazebosim.org), with [ros_control](http://wiki.ros.org/ros_control) controllers and optionally [MoveIt!](https://moveit.ros.org/).
The launch can be parametrized with arguments : 

**Arguments**:
- **start_gazebo** : If this argument is false, Gazebo will not be launched within this launched file. It is useful if you already launched Gazebo yourself and just want to spawn the robot. The default value is **true** (Gazebo will be started).
- **gazebo_gui** : If this argument is false, only the Gazebo Server will be launched. The default value is **true**.
- **start_rviz** : If this argument is true, RViz will be launched. The default value is **true**.
- **x0** : The default X-axis position of the robot in Gazebo. The default value is **0.0**.
- **y0** : The default Y-axis position of the robot in Gazebo. The default value is **0.0**.
- **z0** : The default Z-axis position of the robot in Gazebo. The default value is **0.0**.
- **arm** : Name of your robot arm model. See the `kortex_description/arms` folder to see the available robot models. The default value is **gen3**.
- **gripper** : Name of your robot arm's tool / gripper. See the `kortex_description/grippers` folder to see the available end effector models (or to add your own). The default value is **""**. For Gen3, you can also put **robotiq_2f_85**. For Gen3 lite, you need to put **gen3_lite_2f**.
- **robot_name** : This is the namespace of the arm that is going to be spawned. It defaults to **my_$(arg arm)** (so my_gen3 for arm="gen3").
- **use_trajectory_controller** : If this argument is false, one `JointPositionController` per joint will be launched and the arm will offer a basic ROS Control interface to control every joint individually with topics. If this argument is true, a MoveIt! node will be started for the arm and the arm will offer a `FollowJointTrajectory` interface to control the arm (via a `JointTrajectoryController`). The default value is **true**.
- **use_sim_time** : If this value is true, Gazebo will use simulated time instead of system clock. The default value is **true**.
- **debug** : If this value is true, Gazebo will be launched in debug mode. This option is useful for debugging Gazebo-related issues that won't show in the terminal. The default value is **false**.
- **paused** : If this value is true, Gazebo will be started paused. The default value is **$(arg use_trajectory_controller)** because, when MoveIt! is enabled, Gazebo needs to be started paused to let the controllers initialize.

To launch it with default arguments, run the following command in a terminal : 

`roslaunch kortex_gazebo spawn_kortex_robot.launch`

To launch it with optional arguments, specify the argument name, then ":=", then the value you want. For example, : 

`roslaunch kortex_gazebo spawn_kortex_robot.launch start_rviz:=false x0:=1.0 y0:=2.55 use_trajectory_controller:=false`

You can also have a look at the [roslaunch documentation](http://wiki.ros.org/roslaunch/Commandline%20Tools) for more details.

## Contents

The following is a description of the packages included in this repository.

### kortex_control
This package implements the simulation controllers that control the arm in Gazebo. For more details, please consult the [README](kortex_control/readme.md) from the package subdirectory.

**Note** The `ros_control` controllers for the real arm are not yet implemented and will be in a future release of `ros_kortex`.

### kortex_description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots. For more details, please consult the [README](kortex_description/readme.md) from the package subdirectory.

### kortex_driver
This package implements a ROS node that allows communication between a node and a Kinova Gen3 or Gen3 lite robot. For more details, please consult the [README](kortex_driver/readme.md) from the package subdirectory.

### kortex_examples
This package holds all the examples needed to understand the basics of `ros_kortex`. Most of the examples are written in both C++ and Python. Only the MoveIt! example is available exclusively in Python for now.
A more detailed [description](kortex_examples/readme.md) can be found in the package subdirectory.

### kortex_gazebo
This package contains files to simulate the Kinova Gen3 and Gen3 lite robots in Gazebo. For more details, please consult the [README](kortex_gazebo/readme.md) from the package subdirectory.

### kortex_move_it_config
This metapackage contains the auto-generated MoveIt! files to use the Kinova Gen3 and Gen3 lite arms with the MoveIt! motion planning framework. For more details, please consult the [README](kortex_move_it_config/readme.md) from the package subdirectory.

### third_party
This folder contains the third-party packages we use with the ROS Kortex packages. Currently, it consists of two packages used for the simulation of the Robotiq Gripper in Gazebo. We use [gazebo-pkgs](third_party/gazebo-pkgs/README.md) for grasping support in Gazebo and [roboticsgroup_gazebo_plugins](third_party/roboticsgroup_gazebo_plugins/README.md) to mimic joint support in Gazebo.
