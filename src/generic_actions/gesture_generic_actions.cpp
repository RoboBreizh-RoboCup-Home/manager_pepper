#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <robobreizh_msgs/head_position.h>
#include <robobreizh_msgs/joint_position.h>
#include <robobreizh_msgs/PointToObject.h>
#include <robobreizh_msgs/Float32Array1D.h>
#include <robobreizh_msgs/Float32Array2D.h>

#include "database_model/object_model.hpp"
#include "database_model/database_utils.hpp"
#include "vision_utils.hpp"

#include <boost/thread/thread.hpp>

#include "generic_actions/gesture_generic_actions.hpp"

using namespace std;

namespace robobreizh {
namespace gesture {
namespace generic {

/**
 * @brief Make the robot head move to a specific position
 * @param head_pitch_angle The pitch angle of the head
 * @param head_yaw_angle The yaw angle of the head
 * @param head_pitch_time The time to reach the pitch angle
 * @param head_yaw_time The time to reach the yaw angle
 */
bool look(std::vector<float> head_pitch_angle, std::vector<float> head_yaw_angle, std::vector<float> head_pitch_time,
          std::vector<float> head_yaw_time) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::head_position>("/robobreizh/manipulation/look");

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
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation/look_up");
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
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation/look_down");
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
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation/look_around");
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
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/robobreizh/manipulation/point_in_front");
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
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<robobreizh_msgs::PointToObject>("/robobreizh/manipulation/pointObjectPosition");
  robobreizh_msgs::PointToObject srv;

  srv.request.distance = distance;
  srv.request.point_x = baselink_point.point.x;
  srv.request.point_y = baselink_point.point.y;
  srv.request.point_z = baselink_point.point.z;

  if (client.call(srv)) {
    ROS_INFO("Call to Point Object");
  } else {
    ROS_INFO("Point Object service - ERROR");
    return false;
  }
  return true;
}

bool joint_angles(std::vector<std::string> joint_names, std::vector<std::vector<float>> joint_angles,
                  std::vector<std::vector<float>> time_lists) {
  // check if the input data is valid
  int joint_names_size = joint_names.size();
  if (joint_angles.size() != time_lists.size()) {
    ROS_ERROR("joint_angles and time_lists must have the same size");
    return false;
  }
  if (joint_angles.size() != joint_names_size) {
    ROS_ERROR("joint_angles and joint_names must have the same size");
    return false;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<robobreizh_msgs::joint_position>("/robobreizh/manipulation/joint_angle_speed_srv");

  // filling up request service message
  robobreizh_msgs::joint_position srv;
  srv.request.joint_names = joint_names;
  robobreizh_msgs::Float32Array2D float32_2D_msg;
  // fill joint angles
  for (int i = 0; i < joint_names_size; i++) {
    float32_2D_msg.row[i].col = joint_angles[i];
  }
  srv.request.angle_lists = float32_2D_msg;
  // fill time lists
  for (int i = 0; i < joint_names_size; i++) {
    float32_2D_msg.row[i].col = time_lists[i];
  }
  srv.request.time_lists = float32_2D_msg;

  if (client.call(srv)) {
    ROS_INFO("Call to joint_angle_speed_srv OK");
    return true;
  } else {
    ROS_ERROR("Failed to call service joint_angle_speed_srv");
    return false;
  }

  return true;
}

}  // namespace generic
}  // namespace gesture
}  // namespace robobreizh
