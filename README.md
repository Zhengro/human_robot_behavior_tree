# human_robot_behavior_tree
A ROS package that involves implementations of behavior trees for several human-robot collaboration experiments.

## Table of contents
- [Requirements](#requirements)
- [Installation](#installation)
  - [Installation of behavior_tree_core](#installation-of-behavior_tree_core)
  - [Installation of xdot and rqt_dot](#installation-of-xdot-and-rqt_dot) (optional)
  - [Installation of kinect_tag_detection](#installation-of-kinect_tag_detection)
- [Update and test](#update-and-test)
  - [Update](#update)
  - [Test](#test)
- [Scenarios](#scenarios)

## Requirements
* The [requirements](https://github.com/Zhengro/kinect_tag_detection#requirements) in kinect_tag_detection

## Installation
### Installation of behavior_tree_core
**behavior_tree_core** is a ROS package in the library [ROS-Behavior-Tree](https://github.com/miccol/ROS-Behavior-Tree) that supports behavior tree nodes like fallback (selector), sequence, etc. Some parts of it will be modified to be used in this project. To install it:
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
[kinect_tag_detection](https://github.com/Zhengro/kinect_tag_detection) will be used to publish relevant tag info (i.e., [Usage](https://github.com/Zhengro/kinect_tag_detection#usage) 5) which can be subscribed by the behavior trees to control some of their condition nodes. Follow its installation steps to get it ready.

## Update and test

### Update
Modified versions of node implementations (e.g., [fallback_node_with_memory.cpp](https://github.com/Zhengro/human_robot_behavior_tree/blob/master/behavior_tree_core/src/fallback_node_with_memory.cpp) and [sequence_node_with_memory.cpp](https://github.com/Zhengro/human_robot_behavior_tree/blob/master/behavior_tree_core/src/sequence_node_with_memory.cpp)) and tree implementations (e.g., [behavior_tree.cpp](https://github.com/Zhengro/human_robot_behavior_tree/blob/master/behavior_tree_core/src/behavior_tree.cpp), [tree.cpp](https://github.com/Zhengro/human_robot_behavior_tree/blob/master/behavior_tree_core/src/tree.cpp)) will be used instead of the original ones. They are available in the folder with the same name as before ([behavior_tree_core](https://github.com/Zhengro/human_robot_behavior_tree/tree/master/behavior_tree_core)). Download and update that folder:
```
cd ~/catkin_ws/src/ROS-Behavior-Tree/                              # Navigate to ROS-Behavior-Tree folder
rm -rf ./behavior_tree_core                                        # Remove the original folder
git clone https://github.com/Zhengro/human_robot_behavior_tree.git # Retrieve the updated folder
mv ./human_robot_behavior_tree/behavior_tree_core .                # Update the folder
cd ~/catkin_ws                                                     # Navigate to the workspace
catkin build behavior_tree_core                                    # Build behavior_tree_core
source ~/catkin_ws/devel/setup.bash                                # Source the generated setup file
```
Remember to run the following three lines on every new shell you open or add them to your .bashrc:
```
export ROS_DISTRO=indigo
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash
```
### Test
An example of behavior tree for testing is given by [tree.cpp](https://github.com/Zhengro/human_robot_behavior_tree/blob/master/behavior_tree_core/src/tree.cpp) (It doesn't require to install **kinect_tag_detection**). Follow the steps to run that behavior tree:

1. Open a terminal and make sure that a roscore is up and running:
```
roscore
```
2. Open a second terminal to prepare rqt_dot for visualizing the tree by subscribing to the topic /bt_dotcode:
```
rosrun rqt_dot rqt_dot
```
3. Open a third terminal for executing the tree:
```
rosrun behavior_tree_core tree
```
When test is done, press Ctrl-C to terminate each terminal.

## Scenarios

### Simple scenario: simple packing
#### Only tag info
This is the baseline method in the simple scenario that only uses tag info as conditions for directing robot motions. Thus, a behavior tree with only tag info is built in [BT_Scenario1_OnlyTagInfo.cpp](https://github.com/Zhengro/human_robot_behavior_tree/blob/master/behavior_tree_core/src/BT_Scenario1_OnlyTagInfo.cpp). Follow the first two steps in [Test](#test) and then open a third terminal for executing the tree:
```
rosrun behavior_tree_core BT_Scenario1_OnlyTagInfo
```

#### Both tag info and prediction model
This is our method in the simple scenario that integrates human motion info in the behavior tree to control robot motions more effeciently. The new behavior tree is built in [BT_Scenario1_HumanMotion.cpp](https://github.com/Zhengro/human_robot_behavior_tree/blob/master/behavior_tree_core/src/BT_Scenario1_HumanMotion.cpp). Follow the first two steps in [Test](#test) and then open a third terminal for executing the tree:
```
rosrun behavior_tree_core BT_Scenario1_HumanMotion
```

### Demonstration scenario: packing
To do
