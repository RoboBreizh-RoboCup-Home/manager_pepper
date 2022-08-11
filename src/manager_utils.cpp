#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <vizbox/Story.h>
#include <ros/ros.h>
#include <pnp_ros/names.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

#include "ManagerUtils.hpp"

using namespace std;

namespace robobreizh
{

// declare map of object categories
std::map<std::string, robobreizh::ObjectCategory> object_category{
  // fruits
  { "Apple", robobreizh::ObjectCategory::Fruit },
  { "Fruit", robobreizh::ObjectCategory::Fruit },
  { "Grape", robobreizh::ObjectCategory::Fruit },
  { "Tomato", robobreizh::ObjectCategory::Fruit },
  { "Lemon", robobreizh::ObjectCategory::Fruit },
  { "Banana", robobreizh::ObjectCategory::Fruit },
  { "Orange", robobreizh::ObjectCategory::Fruit },
  { "Coconut", robobreizh::ObjectCategory::Fruit },
  { "Mango", robobreizh::ObjectCategory::Fruit },
  { "Pineapple", robobreizh::ObjectCategory::Fruit },
  { "Grapefruit", robobreizh::ObjectCategory::Fruit },
  { "Pomegranate", robobreizh::ObjectCategory::Fruit },
  { "Watermelon", robobreizh::ObjectCategory::Fruit },
  { "Strawberry", robobreizh::ObjectCategory::Fruit },
  { "Peach", robobreizh::ObjectCategory::Fruit },
  { "Cantaloupe", robobreizh::ObjectCategory::Fruit },
  { "apple", robobreizh::ObjectCategory::Fruit },
  { "banana", robobreizh::ObjectCategory::Fruit },
  { "orange", robobreizh::ObjectCategory::Fruit },
  // vegetables
  { "carrot", robobreizh::ObjectCategory::Vegetable },
  { "broccoli", robobreizh::ObjectCategory::Vegetable },
  // other food
  { "Food", robobreizh::ObjectCategory::OtherFood },
  { "Croissant", robobreizh::ObjectCategory::OtherFood },
  { "Doughnut", robobreizh::ObjectCategory::OtherFood },
  { "Hot dog", robobreizh::ObjectCategory::OtherFood },
  { "Fast food", robobreizh::ObjectCategory::OtherFood },
  { "Popcorn", robobreizh::ObjectCategory::OtherFood },
  { "Cheese", robobreizh::ObjectCategory::OtherFood },
  { "Muffin", robobreizh::ObjectCategory::OtherFood },
  { "Cookie", robobreizh::ObjectCategory::OtherFood },
  { "Dessert", robobreizh::ObjectCategory::OtherFood },
  { "French fries", robobreizh::ObjectCategory::OtherFood },
  { "Baked goods", robobreizh::ObjectCategory::OtherFood },
  { "Pasta", robobreizh::ObjectCategory::OtherFood },
  { "Pizza", robobreizh::ObjectCategory::OtherFood },
  { "Sushi", robobreizh::ObjectCategory::OtherFood },
  { "Bread", robobreizh::ObjectCategory::OtherFood },
  { "Ice cream", robobreizh::ObjectCategory::OtherFood },
  { "Salad", robobreizh::ObjectCategory::OtherFood },
  { "Sandwich", robobreizh::ObjectCategory::OtherFood },
  { "Pastry", robobreizh::ObjectCategory::OtherFood },
  { "Waffle", robobreizh::ObjectCategory::OtherFood },
  { "Pancake", robobreizh::ObjectCategory::OtherFood },
  { "Burrito", robobreizh::ObjectCategory::OtherFood },
  { "Snack", robobreizh::ObjectCategory::OtherFood },
  { "Taco", robobreizh::ObjectCategory::OtherFood },
  { "Hamburger", robobreizh::ObjectCategory::OtherFood },
  { "Cake", robobreizh::ObjectCategory::OtherFood },
  { "Honeycomb", robobreizh::ObjectCategory::OtherFood },
  { "Pretzel", robobreizh::ObjectCategory::OtherFood },
  { "Bagel", robobreizh::ObjectCategory::OtherFood },
  { "Guacamole", robobreizh::ObjectCategory::OtherFood },
  { "Submarine sandwich", robobreizh::ObjectCategory::OtherFood },
  { "sandwich", robobreizh::ObjectCategory::OtherFood },
  { "hot dog", robobreizh::ObjectCategory::OtherFood },
  { "pizza", robobreizh::ObjectCategory::OtherFood },
  { "donut", robobreizh::ObjectCategory::OtherFood },
  { "cake", robobreizh::ObjectCategory::OtherFood },
  { "Candy", robobreizh::ObjectCategory::OtherFood }
};

std::string RoboBreizhManagerUtils::convertCamelCaseToSpacedText(std::string params)
{
  std::string action;
  for (std::string::iterator it = params.begin(); it != params.end(); ++it)
  {
    if (std::isupper(*it))
    {
      action += " ";
      action += std::tolower(*it);
    }
    else
    {
      action += *it;
    }
  }
  return action;
}
// Shamelessly stolen from https://github.com/ros/ros_comm/issues/286
bool RoboBreizhManagerUtils::isPublisherReady(const ros::Publisher& pub)
{
  XmlRpc::XmlRpcValue req, res, payload;
  req[0] = ros::this_node::getName();
  // For more information, check http://wiki.ros.org/ROS/Master_API
  if (!ros::master::execute("getSystemState", req, res, payload, false))
  {
    return false;
  }
  const size_t kSubIndex = 1;
  const auto& subscribers = payload[kSubIndex];
  for (size_t j = 0; j < subscribers.size(); j++)
  {
    const auto& subscriber = subscribers[j];
    // Finds the topic which this node publishes to
    std::ostringstream topic;
    const size_t kTopicNameIndex = 0;
    const size_t kSubscribersIndex = 1;
    subscriber[kTopicNameIndex].write(topic);
    if (topic.str() == pub.getTopic())
    {
      auto real_num_subscribers = subscriber[kSubscribersIndex].size();
      if (pub.getNumSubscribers() == real_num_subscribers)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  }

  // The publisher is ready to publish since the topic does not exist yet
  return true;
}

string RoboBreizhManagerUtils::getPNPConditionStatus()
{
  // TODO: Add try catch if failure
  ros::NodeHandle nh;
  boost::shared_ptr<std_msgs::String const> shared_msg;
  std_msgs::String msg;

  shared_msg = ros::topic::waitForMessage<std_msgs::String>(TOPIC_PNPCONDITION, nh);

  if (shared_msg != NULL)
  {
    return msg.data;
  }
  return "";
}

bool RoboBreizhManagerUtils::setPNPConditionStatus(const string& status)
{
  ROS_INFO("Update PNP condition status to %s", status.c_str());
  std_msgs::String statusMsg;
  statusMsg.data = status;
  ros::Publisher rosPnpPublisher = *pnpPublisher_;
  rosPnpPublisher.publish(statusMsg);
  // RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(TOPIC_PNPCONDITION, statusMsg);
  // RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(TOPIC_PNPCONDITION, statusMsg);
  return true;
}

void RoboBreizhManagerUtils::pubVizBoxRobotText(const std::string& text)
{
  std_msgs::String robotMsg;
  robotMsg.data = text;
  RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>("/robot_text", robotMsg);
}

void RoboBreizhManagerUtils::pubVizBoxOperatorText(const std::string& text)
{
  std_msgs::String operatorMsg;
  operatorMsg.data = text;
  RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>("/operator_text", operatorMsg);
}

void RoboBreizhManagerUtils::pubVizBoxChallengeStep(const uint& challengeStep)
{
  std_msgs::UInt32 challengeUint32;
  challengeUint32.data = challengeStep;
  RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::UInt32>("/challenge_step", challengeUint32);
}

void RoboBreizhManagerUtils::pubVizBoxStory(const vizbox::Story story)
{
  RoboBreizhManagerUtils::sendMessageToTopic<vizbox::Story>("/story", story);
}
}  // namespace robobreizh
