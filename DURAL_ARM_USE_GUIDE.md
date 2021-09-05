# Franka ROS Interface for dual-arm Curiosity-Tabletop

## Installation

Refer [README](README.md). We recommend using `catkin_make` instead of `catkin build`.

After building the package:

- Copy/move the *dual_franka.sh* file to `catkin_ws/`
- The default values in the bash file is comparable with the situation in SMART Lab CUHK, 
  change the values in the copied file as you wish.

## Usage

In `catkin_ws/`, run:

```shell
./dual_franka.launch master
```

Then, inside the activated environment, run:

```shell
roslaunch franka_interface dual_interface.launch  # (use argument load_gripper:=false for starting without gripper)
```

This will activate 2 RViz windows for each of the arm. Note that there is no cross collision checking for now,
so be careful to move the robots.

Available keyword arguments for launch file:

- `load_gripper`: start driver node with the Franka gripper (default: `true`).
- `start_controllers`: load the available controllers to the controller manager (default: `true`).
- `start_moveit`: start moveit server along with the driver node (default: `true`).
- `load_demo_planning_scene`: loads a default planning scene for MoveIt planning with simple objects for collision avoidance (default: `false`). 
   See [create_demo_planning_scene.py](franka_moveit/scripts/create_demo_planning_scene.py).

This starts the robot controllers and drivers to expose a variety of ROS topics and services for communicating with and controlling the robot. 
The robot's measurements and controllers can be accessed using ROS topics and services 
(see below to find out about some available topics and services).
