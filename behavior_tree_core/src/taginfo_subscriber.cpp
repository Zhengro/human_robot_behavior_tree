#include <taginfo_subscriber.h>
#include <behavior_tree_core/TagInfo.h>
#include <cctype>
#include <algorithm>
#include <global_taginfo.h>

namespace BT
{
TagInfoSubscriber::TagInfoSubscriber(const std::string& topic) :
  topic_(topic) {}

TagInfoSubscriber::~TagInfoSubscriber() {}


void chatterCallback(const behavior_tree_core::TagInfo::ConstPtr& msg)
{
  // ROS_INFO("%d", msg->enter_target_areas);
  g_enter_target_areas = msg->enter_target_areas;
  g_small_box = msg->small_box;
  g_stay_still = msg->stay_still;
  g_box_tag_position[0] = msg->box_tag_position.data[0];
  g_box_tag_position[1] = msg->box_tag_position.data[1];
  g_box_tag_position[2] = msg->box_tag_position.data[2];
}


void TagInfoSubscriber::subscribe()
{
  ROS_INFO_STREAM("Start subscribing to the topic: " << topic_);
  taginfo_subscriber_ = n_.subscribe(topic_, 1000, chatterCallback);

  ros::spin();
}
}  // namespace BT
