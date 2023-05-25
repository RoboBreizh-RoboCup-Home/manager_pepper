#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <robobreizh_msgs/PointToObject.h>

#include "database_model/object_model.hpp"
#include "database_model/database_utils.hpp"
#include "vision_utils.hpp"

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
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::PointToObject>("/robobreizh/manipulation_pepper/pointObjectPosition");
  robobreizh_msgs::PointToObject srv;

  robobreizh::database::ObjectModel om;
  database::Object object = om.getLastObject();
  

  geometry_msgs::Point baselink_point;
  baselink_point = robobreizh::convertOdomToBaseLink(object.position.x,object.position.y, object.position.z);

  // srv.request.point_x = baselink_point.x;
  // srv.request.point_y = baselink_point.y;
  // srv.request.point_z = baselink_point.z;

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
