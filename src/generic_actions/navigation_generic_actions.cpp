#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalID.h>
#include <navigation_pep/NavigationDestination.h>
#include <navigation_pep/AngleSrv.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Pose.h"
#include <navigation_pep/InitPose.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <boost/thread/thread.hpp>
#include "database_model/object_model.hpp"
#include "manager_utils.hpp"
#include "generic_actions/navigation_generic_actions.hpp"

using namespace std;

namespace robobreizh {
namespace navigation {
namespace generic {

// declare global variable
std::vector<actionlib_msgs::GoalStatus> g_status;

/**
 * Sends an empty message to /move_base/cancel to stop the current Goal
 */
bool cancelGoal() {
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10, false);
  actionlib_msgs::GoalID msg;
  msg.stamp.now();
  ROS_INFO("Cancelling navigation");
  pub.publish(msg);
  ros::spinOnce();
  return true;
}

/**
 * Service callback for getStatus()
 */
void getStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  g_status = msg->status_list;
}

/**
 * Listen to /move_base/status
 * and parse actionlib_msgs/GoalStatus.msg
 * @return int status
 */
std::vector<actionlib_msgs::GoalStatus> getStatus() {
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/move_base/status", 10, getStatusCallback);
  ros::spinOnce();
  return g_status;
}

/**
 * returns if move base received a cancel message
 */
bool isMoveBaseGoal() {
  auto status = getStatus();
  for (actionlib_msgs::GoalStatus cur : status) {
    if (cur.status == cur.RECALLED || cur.status == cur.RECALLING) {
      return false;
    }
  }
  return true;
}

bool setInitPose(geometry_msgs::PoseWithCovarianceStamped p) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<navigation_pep::InitPose>("/robobreizh/navigation_pepper/set_init_pose");
  navigation_pep::InitPose srv;

  srv.request.pose = p;

  if (client.call(srv)) {
    ROS_INFO("Init pose done");
  } else {
    ROS_ERROR("Failed to call service set_init_pose");
    return false;
  }
  return true;
}

bool moveTowardsObject(string objectName) {
  robobreizh::database::ObjectModel om;
  auto obj_position = om.getPositionByLabel(objectName);
  moveTowardsPosition(obj_position.position, 0.0);
  return true;
}

bool moveTowardsPosition(geometry_msgs::Point p, float angle) {
  ros::NodeHandle nh;

  geometry_msgs::Pose destination;
  destination.position = p;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, angle);
  orientation.normalize();
  tf2::convert(orientation, destination.orientation);

  ROS_INFO("Sending goal ROS Position(%f,%f,%f), Orientation(%f,%f,%f,%f)", destination.position.x,
           destination.position.y, destination.position.z, destination.orientation.w, destination.orientation.x,
           destination.orientation.y, destination.orientation.z);

  ros::ServiceClient client =
      nh.serviceClient<navigation_pep::NavigationDestination>("/robobreizh/navigation_pepper/move_to_goal");
  navigation_pep::NavigationDestination srv;
  srv.request.pose = destination;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Navigation success: Goal achieved");
    } else {
      ROS_ERROR("Navigation timed out");
      return false;
    }
  } else {
    ROS_ERROR("Failed to call service move_to_goal");
    return false;
  }
  return true;
}
bool moveTowardsPosition(geometry_msgs::Pose p) {
  ros::NodeHandle nh;

  ROS_INFO("Sending goal ROS Position(%f,%f,%f), Orientation(%f,%f,%f,%f)", p.position.x, p.position.y, p.position.z,
           p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);

  ros::ServiceClient client =
      nh.serviceClient<navigation_pep::NavigationDestination>("/robobreizh/navigation_pepper/move_to_goal");
  navigation_pep::NavigationDestination srv;
  srv.request.pose = p;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Navigation success: Goal achieved");
    } else {
      ROS_ERROR("Navigation timed out");
      return false;
    }
  } else {
    ROS_ERROR("Failed to call service move_to_goal");
    return false;
  }
  return true;
}

bool rotateOnPoint(float angle) {
  ros::NodeHandle nh;

  ros::ServiceClient client =
      nh.serviceClient<navigation_pep::AngleSrv>("/robobreizh/navigation_pepper/rotate_on_point");
  navigation_pep::AngleSrv srv;
  std::cout << std::to_string(angle) << std::endl;
  srv.request.angle = angle;
  std::cout << std::to_string(srv.request.angle) << std::endl;

  if (client.call(srv)) {
    ROS_INFO("Rotation done");
  } else {
    ROS_ERROR("Failed to call service rotation_on_point");
    return false;
  }
  return true;
}
/**
 * Service callback for getStatus()
 */
void getCurrentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  g_current_position = msg->pose;
}

/**
 * Listen to /amcl_pose
 * @return PoseWithCovarianceStamped
 */
geometry_msgs::PoseWithCovariance getCurrentPosition() {
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/amcl_pose", 1, getCurrentPoseCallback);
  ros::spinOnce();
  return g_current_position;
}
}  // namespace generic
}  // namespace navigation
}  // namespace robobreizh
