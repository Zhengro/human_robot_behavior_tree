#ifndef TAGINFO_SUBSCRIBER_H
#define TAGINFO_SUBSCRIBER_H

#include <string>
#include <ros/ros.h>
#include <tree_node.h>
#include <boost/algorithm/string.hpp>

namespace BT
{

class TagInfoSubscriber
{
public:
  
  explicit TagInfoSubscriber(const std::string& topic = "/tag_info");

  ~TagInfoSubscriber();

  void subscribe();

private:
  
  // A node handle used by the ROS subscriber TagInfoSubscriber::taginfo_subscriber_.
  ros::NodeHandle n_;

  ros::Subscriber taginfo_subscriber_;

  std::string topic_;

};
}  // namespace BT

#endif  // TAGINFO_SUBSCRIBER_H
