#include <std_msgs/String.h>
#include <ros/ros.h>

#include "plan_high_level_actions/manipulation_plan_actions.hpp"
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

void aGraspObject(std::string params, bool* run)
{
  // Get Parameters
  int i_object = params.find("_");
  int i_hand = params.find("_", i_object + 1);
  string object = params.substr(0, i_object);
  string hand = params.substr(i_object + 1, i_hand); // Left, Right, Both

  ROS_INFO("aGraspObject - grasp object %s using %s hand", object.c_str(), hand.c_str());

  // CV - Find and localise handle
  // Manipulation - Grasp handle  => Both actions should be on separate modules, with one to obtain object position,
  // used in the following.
  //manipulation::generic::grabHandle(object, hand);

  RoboBreizhManagerUtils::setPNPConditionStatus("GraspOK");
  *run = 1;
}

void aPutObject(std::string params, bool* run)
{
   // Get Parameters
  string hand = params;

  ROS_INFO("aPutObject - put object in %s hand", hand.c_str()); // Left, Right, Both

  // Manipulation - Put object held on a certain hand on a certain surface position

  RoboBreizhManagerUtils::setPNPConditionStatus("PutOK");
  *run = 1;
}

void aPourObject(std::string params, bool* run)
{
  // Get Parameters
  int i_hand = params.find("_");
  int i_target = params.find("_", i_hand + 1);
  string hand = params.substr(0, i_hand); // Left, Right, Both
  string target = params.substr(i_hand + 1, i_target);

  ROS_INFO("aPourObject - pour object of %s hand to target %s", hand.c_str(), target.c_str());

  RoboBreizhManagerUtils::setPNPConditionStatus("PourOK");
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

void aPullObject(std::string params, bool* run)
{
  // Grasp and Pull object
  int i_hand = params.find("_");
  int i_target = params.find("_", i_hand + 1);
  string hand = params.substr(0, i_hand); // Left, Right, Both
  string target = params.substr(i_hand + 1, i_target);

  ROS_INFO("aPullObject - Pull object %s using %s hand(s)", hand.c_str(), target.c_str());

  RoboBreizhManagerUtils::setPNPConditionStatus("PullOK");
  *run = 1;

}

}  // namespace plan
}  // namespace manipulation
}  // namespace robobreizh
