#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <ros/ros.h>
#include <pnp_ros/names.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

#include "manager_utils.hpp"
#include <visualization_msgs/Marker.h>

using namespace std;

namespace robobreizh {

std::string toLower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  return str;
}

std::string convertCamelCaseToSpacedText(std::string params) {
  std::string action;
  for (std::string::iterator it = params.begin(); it != params.end(); ++it) {
    if (std::isupper(*it)) {
      action += " ";
      action += std::tolower(*it);
    } else {
      action += *it;
    }
  }
  return action;
}
// Shamelessly stolen from https://github.com/ros/ros_comm/issues/286
bool RoboBreizhManagerUtils::isPublisherReady(const ros::Publisher& pub) {
  XmlRpc::XmlRpcValue req, res, payload;
  req[0] = ros::this_node::getName();
  // For more information, check http://wiki.ros.org/ROS/Master_API
  if (!ros::master::execute("getSystemState", req, res, payload, false)) {
    return false;
  }
  const size_t kSubIndex = 1;
  const auto& subscribers = payload[kSubIndex];
  for (size_t j = 0; j < subscribers.size(); j++) {
    const auto& subscriber = subscribers[j];
    // Finds the topic which this node publishes to
    std::ostringstream topic;
    const size_t kTopicNameIndex = 0;
    const size_t kSubscribersIndex = 1;
    subscriber[kTopicNameIndex].write(topic);
    if (topic.str() == pub.getTopic()) {
      auto real_num_subscribers = subscriber[kSubscribersIndex].size();
      if (pub.getNumSubscribers() == real_num_subscribers) {
        return true;
      } else {
        return false;
      }
    }
  }

  // The publisher is ready to publish since the topic does not exist yet
  return true;
}

string RoboBreizhManagerUtils::getPNPConditionStatus() {
  // TODO: Add try catch if failure
  ros::NodeHandle nh;
  boost::shared_ptr<std_msgs::String const> shared_msg;
  std_msgs::String msg;

  shared_msg = ros::topic::waitForMessage<std_msgs::String>(TOPIC_PNPCONDITION, nh);

  if (shared_msg != NULL) {
    return msg.data;
  }
  return "";
}

bool RoboBreizhManagerUtils::setPNPConditionStatus(const string& status) {
  ROS_INFO("Update PNP condition status to %s", status.c_str());
  std_msgs::String statusMsg;
  statusMsg.data = status;
  ros::Publisher rosPnpPublisher = *pnpPublisher_;
  rosPnpPublisher.publish(statusMsg);
  // RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(TOPIC_PNPCONDITION, statusMsg);
  // RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(TOPIC_PNPCONDITION, statusMsg);
  return true;
}

void RoboBreizhManagerUtils::pubVizBoxRobotText(const std::string& text) {
  std_msgs::String robotMsg;
  robotMsg.data = text;
  RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>("/robot_text", robotMsg);
}

void RoboBreizhManagerUtils::pubVizBoxOperatorText(const std::string& text) {
  std_msgs::String operatorMsg;
  operatorMsg.data = text;
  RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>("/operator_text", operatorMsg);
}

void RoboBreizhManagerUtils::publishPersonMarkers(const std::vector<robobreizh::database::Person> &persons) {
  ros::NodeHandle nh;
  ros::Publisher m_visualization_pub = nh.advertise<visualization_msgs::Marker>("manager_pepper/person_markers", 10);
  auto person_markers{ std::vector<visualization_msgs::Marker>{} };

  for (auto person : persons) {
    if (person.distance() > 0) {
      // person marker
      visualization_msgs::Marker marker{};
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "persons";
      marker.id = person.id;
      marker.type = visualization_msgs::Marker::POINTS;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration{500};
      marker.pose.position.x = person.pose().pose.position.x;
      marker.pose.position.y = person.pose().pose.position.y;
      marker.pose.position.z = person.pose().pose.position.z;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.5;

      person_markers.emplace_back(marker);
    }
  }
  // publish markers
  for (const auto& m : person_markers)
    m_visualization_pub.publish(m);
}

}  // namespace robobreizh
