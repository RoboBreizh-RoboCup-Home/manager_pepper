#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <robobreizh_msgs/head_position.h>
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

bool look(std::vector<float> head_pitch_angle, std::vector<float> head_yaw_angle, std::vector<float> head_pitch_time,
          std::vector<float> head_yaw_time) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::head_position>("/robobreizh/manipulation_pepper/look");

  robobreizh_msgs::head_position srv;
  srv.request.head_pitch_angle = head_pitch_angle;
  srv.request.head_yaw_angle = head_yaw_angle;

  srv.request.head_pitch_time = head_pitch_time;
  srv.request.head_yaw_time = head_yaw_time;

  if (client.call(srv)) {
    ROS_INFO("Call to looking service  OK");
    return true;
  } else {
    ROS_ERROR("Failed to call service look");
  }
  return false;
}

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
    ROS_INFO("Call to point in front");
  } else {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}

bool pointObjectPosition(geometry_msgs::PointStamped baselink_point, float distance) {
  ROS_INFO("Pointing Object");
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<robobreizh_msgs::PointToObject>("/robobreizh/manipulation/pointObjectPosition");
  robobreizh_msgs::PointToObject srv;

  srv.request.distance = distance;
  srv.request.point_x = baselink_point.point.x;
  srv.request.point_y = baselink_point.point.y;
  srv.request.point_z = baselink_point.point.z;

  std::cout << "PointObjectPosition" << endl;
  std::cout << (std::to_string(distance)) << endl;
  std::cout << (std::to_string(baselink_point.point.z)) << endl;

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
