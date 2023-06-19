#include <ros/ros.h>
#include <std_msgs/String.h>

#include "plan_high_level_actions/gesture_plan_actions.hpp"
#include "generic_actions/gesture_generic_actions.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"
#include <std_msgs/Int32.h>
#include "database_model/person_model.hpp"
#include <geometry_msgs/PointStamped.h>

using namespace std;

namespace robobreizh {
namespace gesture {
namespace plan {
void aLookAt(string params, bool* run) {
  *run = 1;
}
void aLook(std::string params, bool* run) {
  ROS_INFO("looking %s", params.c_str());
  if (params == "Up") {
    // manipulation::generic::lookUp();
    system("rosservice call /robobreizh/manipulation/look_up");
  } else if (params == "Down") {
    system("rosservice call /robobreizh/manipulation/look_down");
    // manipulation::generic::lookDown();
  } else if (params == "Around") {
    system("rosservice call /robobreizh/manipulation/look_around");
    // manipulation::generic::lookAround();
  } else if (params == "Left") {
    system("rosservice call /robobreizh/manipulation/look_left");
  } else if (params == "Right") {
    system("rosservice call /robobreizh/manipulation/look_right");
  } else if (params == "DownStickler") {
    robobreizh::gesture::generic::look({ 0.0f }, { 0.0 }, { 1.0 }, { 1.0 });
  } else if (params == "Shoes") {
    robobreizh::gesture::generic::look({ 25.5f }, { 0.0 }, { 1.0 }, { 1.0 });
  }
}

void aPointAt(std::string params, bool* run) {
  if (params == "RuleBreaker") {
    std_msgs::Int32 stickler_tracked_person;
    SQLiteUtils::getParameterValue<std_msgs::Int32>("stickler_tracker_person_name", stickler_tracked_person);
    robobreizh::database::PersonModel pm;
    ROS_INFO_STREAM("trying to get person with id " << stickler_tracked_person.data);
    robobreizh::database::Person person = pm.getPerson(stickler_tracked_person.data);
    geometry_msgs::PointStamped ps;
    ps.point.x = person.position.x;
    ps.point.y = person.position.y;
    ps.point.z = person.position.z;
    ps.header.frame_id = "map";
    robobreizh::gesture::generic::pointObjectPosition(ps, person.distance);
  }
  // manipulation::generic::pointInFront();
}

void aPointObject(std::string params, bool* run) {
  ROS_INFO("pointing Objects");
  system("rosservice call /robobreizh/manipulation/pointObjectPosition");
}

}  // namespace plan
}  // namespace gesture
}  // namespace robobreizh
