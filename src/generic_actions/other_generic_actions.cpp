#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>
#include <vector>
#include <string>
#include <boost/thread/thread.hpp>

#include "generic_actions/other_generic_actions.hpp"
#include "database_model/person_model.hpp"
#include "vision_utils.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"


using namespace std;

namespace robobreizh {
namespace other {
namespace generic {
bool waitForGoSignal() {
  ros::NodeHandle nh;
  boost::shared_ptr<std_msgs::String const> shared_msg;
  std_msgs::String msg;
  ROS_INFO("wait_for_go_signal - Waiting for go signal from /robobreizh/manager/go");

  shared_msg = ros::topic::waitForMessage<std_msgs::String>("/robobreizh/manager/go", nh);

  if (shared_msg != NULL) {
    msg = *shared_msg;
    ROS_INFO("waitForGoSignal - Let's go! %s", msg.data.c_str());
    return true;
  } else {
    ROS_INFO("waitForGoSignal - ERROR");
    return false;
  }
}

bool findWhoBreakTheRules(int* person_id, int* result) {
  // get person in db and look which rule is broken otherwise turn
  robobreizh::database::PersonModel pm;
  auto persons = pm.getPersons();
  ROS_INFO_STREAM("Number of person in the database: " << persons.size());
  // publish person marker on rviz
  robobreizh:RoboBreizhManagerUtils manager_utils;
  manager_utils.publishPersonMarkers(persons);
  std_msgs::Int32 fr_attempt;

  for (auto person : persons) {
    if (robobreizh::isInForbiddenRoom(person.position.x, person.position.y)) {
      ROS_INFO_STREAM("Person id in forbidden room : " << person.id);
      *person_id = person.id;
      *result = 3;
      fr_attempt.data++;
      if (fr_attempt.data == 1){
        SQLiteUtils::storeNewParameter<std_msgs::Int32>("forbidden_room_attempt", fr_attempt);
      }
      else{
        SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("forbidden_room_attempt", fr_attempt);
      }
      return true;
    }
    if (!person.is_drink) {
      ROS_INFO_STREAM("Person id without drink : " << person.id);
      *person_id = person.id;
      *result = 2;
      return true;
    }
    if (person.is_shoes) {
      ROS_INFO_STREAM("Person id with shoes : " << person.id);
      *person_id = person.id;
      *result = 1;
      return true;
    }
  }
  *result = 0;
  return false;
}

}  // namespace generic
}  // namespace other
}  // namespace robobreizh
