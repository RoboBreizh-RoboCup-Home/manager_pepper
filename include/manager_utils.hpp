#ifndef _PNP_ROBOBREIZH_MANAGER_UTILS_
#define _PNP_ROBOBREIZH_MANAGER_UTILS_

#include <std_msgs/String.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_ros/buffer.h>

extern uint8_t g_guest_counter;
extern uint8_t g_guest_limit;
extern uint8_t g_name_failure_counter;
extern uint8_t g_drink_failure_counter;
extern uint8_t g_failure_counter;
extern uint8_t g_failure_limit;
extern std::string g_default_name;
extern std::string g_default_drink;
extern uint8_t g_order_index;
extern uint8_t g_nb_action;
extern geometry_msgs::PoseWithCovariance g_current_position;

const tf2_ros::Buffer tfBuffer{};

namespace robobreizh {

// void pubPlanState(std::string state);
std::string toLower(std::string str);
std::string convertCamelCaseToSpacedText(std::string params);

class RoboBreizhManagerUtils {
public:
  RoboBreizhManagerUtils() = default;
  ~RoboBreizhManagerUtils() = default;

  static ros::Publisher* pnpPublisher_;
  static ros::NodeHandle* rosHandle_;

  static bool isPublisherReady(const ros::Publisher& pub);

  template <typename T>
  static bool sendMessageToTopic(const std::string& topicPath, const T& obj) {
    // TODO: Add Try catch if failure
    // TODO: Add timeout
    ros::NodeHandle handle;
    ros::Publisher topicPublisher = handle.advertise<T>(topicPath, 1000);

    while (ros::ok() && !RoboBreizhManagerUtils::isPublisherReady(topicPublisher)) {
      ROS_INFO("Waiting for publisher to be ready");
      ros::Duration(0.1).sleep();
    }

    if (ros::ok()) {
      topicPublisher.publish(obj);
      ROS_INFO("Sending information to topic %s.", topicPath.c_str());
      topicPublisher.shutdown();
      return true;
    }

    return false;
  }

  template <typename T>
  static bool waitForMessageFromTopic(const std::string& topicPath, T& returnedMessage) {
    // TODO: Add try catch if failure
    ros::NodeHandle nh;
    boost::shared_ptr<T const> shared_msg;  // *shared_msg;

    shared_msg = ros::topic::waitForMessage<T>(topicPath, nh);

    if (shared_msg != NULL) {
      returnedMessage = *shared_msg;
      return true;
    } else {
      return false;
    }
  }

  static std::string getPNPConditionStatus();
  static bool setPNPConditionStatus(const std::string& status);
  static void pubVizBoxRobotText(const std::string& text);
  static void pubVizBoxOperatorText(const std::string& text);
};

enum ObjectCategory { Fruit, Vegetable, OtherFood };

// declare map of object categories
static std::map<std::string, robobreizh::ObjectCategory> object_category{
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
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_MANAGER_UTILS_
