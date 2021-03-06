cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree_core)



find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  roslaunch
  genmsg
)


add_action_files(
  DIRECTORY action
  FILES BT.action
)


add_message_files(
  FILES
  TagInfo.msg
  HumanMotionInfo.msg
)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs # Or other packages containing msgs
)


add_definitions(-Wall -lglut -lGL -std=c++0x)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)



#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#########################################################
# FIND GTest
#########################################################
find_package(GTest)
include_directories(${GTEST_INCLUDE_DIRS})


INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} include)

file(GLOB_RECURSE BTHeadLibrary include/*.h)

set(BTSrcLibrary
src/action_node.cpp
src/behavior_tree.cpp
src/condition_node.cpp
src/control_node.cpp
src/decorators/negation_node.cpp
#src/decorator_retry_node.cpp
#src/decorator_negation_node.cpp
src/draw.cpp
src/exceptions.cpp
src/leaf_node.cpp
src/tick_engine.cpp
src/parallel_node.cpp
src/fallback_node.cpp
src/sequence_node.cpp
src/decorator_node.cpp
src/fallback_node_with_memory.cpp
src/sequence_node_with_memory.cpp
src/tree_node.cpp
src/actions/action_test_node.cpp
src/conditions/condition_test_node.cpp
src/actions/action_node_put_object.cpp
src/actions/action_node_pick_small_object.cpp
src/actions/action_node_pick_big_object.cpp
src/actions/action_node_left_ready_pose.cpp
src/actions/action_node_right_ready_pose.cpp
src/conditions/condition_node_enter_target_areas.cpp
src/conditions/condition_node_stay_still.cpp
src/conditions/condition_node_small_box.cpp
src/conditions/condition_node_big_box.cpp
src/conditions/condition_node_predict_steady_box_size.cpp
src/conditions/condition_node_predict_steady_box_position.cpp
src/conditions/condition_node_predict_big_box.cpp
src/conditions/condition_node_predict_small_box.cpp
src/conditions/condition_node_predict_box_on_left.cpp
src/conditions/condition_node_predict_box_on_right.cpp
src/actions/ros_action.cpp
src/conditions/ros_condition.cpp
src/dot_bt.cpp
src/info_subscriber.cpp
src/global_info.cpp
)

# Compile the core library with name ${PROJECT_NAME}=behavior_tree_core
# You can create executables which target to this library for using BTs
add_library(${PROJECT_NAME} ${BTSrcLibrary} ${BTHeadLibrary})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
add_dependencies(${PROJECT_NAME} behavior_tree_core_generate_messages_cpp)

add_executable(BT_Scenario1_OnlyTagInfo src/BT_Scenario1_OnlyTagInfo.cpp)
target_link_libraries(BT_Scenario1_OnlyTagInfo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})

add_executable(BT_Scenario1_HumanMotion src/BT_Scenario1_HumanMotion.cpp)
target_link_libraries(BT_Scenario1_HumanMotion
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})

add_executable(listener src/listener.cpp)
target_link_libraries(listener 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})
add_dependencies(listener behavior_tree_core_generate_messages_cpp)

add_executable(tree src/tree.cpp)
target_link_libraries(tree
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})

add_executable(gtest_tree src/gtest/gtest_tree.cpp)
target_link_libraries(gtest_tree
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${GTEST_LIBRARIES})

add_executable(gtest_ros src/gtest/external_ros_nodes_test.cpp)
target_link_libraries(gtest_ros
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${GTEST_LIBRARIES})


#add_executable(ros_test src/ros_test.cpp ${BTSrcLibrary} ${BTHeadLibrary})
#target_link_libraries(ros_test ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
#add_dependencies(ros_test behavior_tree_core_generate_messages_cpp)


