#include <std_msgs/String.h>
#include <ros/ros.h>

#include "plan_highLevel_actions/manipulation_plan_actions.hpp"
#include "generic_actions/manipulation_generic_actions.hpp"
#include "manager_utils.hpp"

using namespace std;

namespace robobreizh
{
namespace manipulation
{
namespace plan
{
void aGrabHandle(std::string params, bool* run)
{
  // Get Parameters
  int i_object = params.find("_");
  int i_hand = params.find("_", i_object + 1);
  string object = params.substr(0, i_object);
  string hand = params.substr(i_object + 1, i_hand);

  ROS_INFO("aGrabHandle - grab object %s using %s hand", object.c_str(), hand.c_str());

  // CV - Find and localise handle
  // Manipulation - Grasp handle  => Both actions should be on separate modules, with one to obtain object position,
  // used in the following.
  manipulation::generic::grabHandle(object, hand);
  *run = 1;
}

void aDropObject(std::string params, bool* run)
{
  // Get Parameters
  string hand = params;

  ROS_INFO("aDropObject - drop object in %s hand", hand.c_str());

  // Manipulation - Put object held on a certain hand on a certain position
  manipulation::generic::dropObject(hand);
}

void aLook(std::string params, bool* run)
{
  ROS_INFO("looking %s", params.c_str());
  if (params == "Up")
  {
    // manipulation::generic::lookUp();
    system("rosservice call /robobreizh/manipulation/look_up");
  }
  else if (params == "Down")
  {
    system("rosservice call /robobreizh/manipulation/look_down");
    // manipulation::generic::lookDown();
  }
  else if (params == "Around")
  {
    system("rosservice call /robobreizh/manipulation/look_around");
    // manipulation::generic::lookAround();
  }
  else if (params == "Left")
  {
    system("rosservice call /robobreizh/manipulation/look_left");
  }
  else if (params == "Right")
  {
    system("rosservice call /robobreizh/manipulation/look_right");
  }
  else if (params == "DownStickler")
  {
    system("rosservice call /robobreizh/manipulation/look_above_wall");
    // manipulation::generic::lookAround();
  }
}

void aPointAt(std::string params, bool* run)
{
  ROS_INFO("point in front");
  system("rosservice call /robobreizh/manipulation/point_in_front");
  // manipulation::generic::pointInFront();
}
void aBendArms(string params, bool* run)
{
  if (params == "Right")
  {
    // bend the Right arm by 90 degree to hold the bag
  }
  else if (params == "Left")
  {
    // bend the left arm by 90 degree to hold the bag
  }
  // bend the arm by 90 degree to hold the bag
  RoboBreizhManagerUtils::setPNPConditionStatus("Done");
  *run = 1;
}

}  // namespace plan
}  // namespace manipulation
}  // namespace robobreizh
