# human_robot_behavior_tree
A ROS package that involves implementations of behavior trees for several human-robot collaboration experiments.

## Table of contents
- [Requirements](#requirements)
- [Installation](#installation)
  - [Installation of behavior_tree_core](#installation-of-behavior_tree_core)
  - [Installation of xdot and rqt_dot](#installation-of-xdot-and-rqt_dot) (optional)
  - [Installation of kinect_tag_detection](#installation-of-kinect_tag_detection)
- [Usage](#usage)

## Requirements
* To Do

## Installation
### Installation of behavior_tree_core
[behavior_tree_core](https://github.com/miccol/ROS-Behavior-Tree) is a ROS behavior tree library that supports behavior tree nodes like fallback (selector), sequence, etc. Some parts of it will be reused with some modifications in this package. To install it:
```
export ROS_DISTRO=indigo                                           # Set this to your distro
source /opt/ros/$ROS_DISTRO/setup.bash                             # Source your ROS distro 
mkdir -p ~/catkin_ws/src                                           # Make a new catkin workspace if no one exists
cd ~/catkin_ws/src                                                 # Navigate to the source space
git clone https://github.com/miccol/ROS-Behavior-Tree.git          # Retrieve its source code
cd ~/catkin_ws                                                     # Navigate to the workspace
catkin build behavior_tree_core                                    # Build behavior_tree_core
source ~/catkin_ws/devel/setup.bash                                # Source the generated setup file
```

### Installation of xdot and rqt_dot
It is optional to install [xdot](https://github.com/jbohren/xdot) and [rqt_dot](https://github.com/jbohren/rqt_dot) for visualizing the tree.
```
cd ~/catkin_ws/src                                                 # Navigate to the source space
git clone https://github.com/jbohren/xdot.git                      # Retrieve the source code
git clone https://github.com/jbohren/rqt_dot.git                   # Retrieve the source code
cd ~/catkin_ws                                                     # Navigate to the workspace
catkin build xdot                                                  # Build xdot
catkin build rqt_dot                                               # Build rqt_dot
source ~/catkin_ws/devel/setup.bash                                # Source the generated setup file
```

### Installation of kinect_tag_detection
Some condition nodes of the behavior trees can be formed with information from [kinect_tag_detection](https://github.com/Zhengro/kinect_tag_detection). Follow the link to get it ready.

## Usage
Modified versions of node implementations will be used. They are available in the folder [behavior_tree_core](). Download and update the corresponding folder. 







