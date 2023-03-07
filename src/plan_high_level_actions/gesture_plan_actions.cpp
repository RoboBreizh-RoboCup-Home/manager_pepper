#include <ros/ros.h>
#include <std_msgs/String.h>

#include "plan_high_level_actions/gesture_plan_actions.hpp"
#include "generic_actions/gesture_generic_actions.hpp"
#include "manager_utils.hpp"

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
    system("rosservice call /robobreizh/manipulation/look_above_wall");
    // manipulation::generic::lookAround();
  }
}

void aPointAt(std::string params, bool* run) {
  ROS_INFO("point in front");
  system("rosservice call /robobreizh/manipulation/point_in_front");
  // manipulation::generic::pointInFront();
}

}  // namespace plan
}  // namespace gesture
}  // namespace robobreizh
