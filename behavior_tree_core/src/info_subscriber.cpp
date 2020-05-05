#include <info_subscriber.h>
#include <behavior_tree_core/TagInfo.h>
#include <behavior_tree_core/HumanMotionInfo.h>
#include <cctype>
#include <algorithm>
#include <global_info.h>

namespace BT
{
InfoSubscriber::InfoSubscriber(const std::string& topic1, const std::string& topic2) :
  topic1_(topic1),
  topic2_(topic2) {}

InfoSubscriber::~InfoSubscriber() {}

void tagCallback(const behavior_tree_core::TagInfo::ConstPtr& msg)
{
  // ROS_INFO("%d", msg->enter_target_areas);
  g_enter_target_areas = msg->enter_target_areas;
  g_small_box = msg->small_box;
  g_stay_still = msg->stay_still;
  g_box_tag_position[0] = msg->box_tag_position.data[0];
  g_box_tag_position[1] = msg->box_tag_position.data[1];
  g_box_tag_position[2] = msg->box_tag_position.data[2];
}

void humanmotionCallback(const behavior_tree_core::HumanMotionInfo::ConstPtr& msg)
{
  // ROS_INFO("%d", msg->predict_steady_box_size);
  g_predict_steady_box_size = msg->predict_steady_box_size;
  g_predict_steady_box_position = msg->predict_steady_box_position;
  g_predict_small_box = msg->predict_small_box;
  g_predict_box_on_left = msg->predict_box_on_left;
}

void InfoSubscriber::subscribe()
{
  ROS_INFO_STREAM("Start subscribing to the topic: " << topic1_);
  taginfo_subscriber_ = n_.subscribe(topic1_, 1000, tagCallback);

  ROS_INFO_STREAM("Start subscribing to the topic: " << topic2_);
  humanmotioninfo_subscriber_ = n_.subscribe(topic2_, 1000, humanmotionCallback);

  ros::spin();
}
}  // namespace BT
