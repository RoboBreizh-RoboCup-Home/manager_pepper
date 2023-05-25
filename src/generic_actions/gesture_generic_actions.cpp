#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include "database_model/object_model.hpp"
#include "database_model/database_utils.hpp"
#include <boost/thread/thread.hpp>

#include "generic_actions/gesture_generic_actions.hpp"

using namespace std;

namespace robobreizh {
namespace gesture {
namespace generic {

bool lookUp() {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation_pepper/look_up");
  std_srvs::Empty srv;

  if (client.call(srv)) {
    ROS_INFO("Call to looking up OK");
  } else {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}
bool lookDown() {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation_pepper/look_down");
  std_srvs::Empty srv;

  if (client.call(srv)) {
    ROS_INFO("Call to looking up OK");
  } else {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}
bool lookAround() {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation_pepper/look_around");
  std_srvs::Empty srv;

  if (client.call(srv)) {
    ROS_INFO("Call to looking up OK");
  } else {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}

bool pointInFront() {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation_pepper/point_in_front");
  std_srvs::Empty srv;

  if (client.call(srv)) {
    ROS_INFO("Call to looking up OK");
  } else {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}

bool pointObjectPosition(){
  ROS_INFO("Pointing Object");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation_pepper/pointObjectPosition");
  std_srvs::Empty srv;

  robobreizh::database::ObjectModel om;
  database::Object object = om.getLastObject();
  float distance = object.distance;
  float object_pos_x = object.position.x;
  float object_pos_y = object.position.y;
  float object_pos_z = object.position.z;

  if (client.call(srv)) {
    ROS_INFO("Call to Point Object");
  } else {
    ROS_INFO("Point Object service - ERROR");
    return false;
  }
  return true;
}

}  // namespace generic
}  // namespace gesture
}  // namespace robobreizh
