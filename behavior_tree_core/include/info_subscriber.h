#ifndef INFO_SUBSCRIBER_H
#define INFO_SUBSCRIBER_H

#include <string>
#include <ros/ros.h>
#include <tree_node.h>
#include <boost/algorithm/string.hpp>

namespace BT
{

class InfoSubscriber
{
public:
  
  explicit InfoSubscriber(const std::string& topic1 = "/tag_info", const std::string& topic2 = "/human_motion_info");

  ~InfoSubscriber();

  void subscribe();

private:
  
  // A node handle
  ros::NodeHandle n_;

  ros::Subscriber taginfo_subscriber_;
  ros::Subscriber humanmotioninfo_subscriber_;

  std::string topic1_;
  std::string topic2_;

};
}  // namespace BT

#endif  // INFO_SUBSCRIBER_H
