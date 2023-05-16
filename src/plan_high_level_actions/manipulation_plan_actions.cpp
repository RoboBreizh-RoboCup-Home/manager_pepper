#include <std_msgs/String.h>
#include <ros/ros.h>

#include "plan_high_level_actions/manipulation_plan_actions.hpp"
#include "generic_actions/manipulation_generic_actions.hpp"
#include "manager_utils.hpp"
#include "manipulation_utils.hpp"
#include "database_model/object_model.hpp"

#include <manipulation_pepper/MovementAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

namespace robobreizh {
namespace manipulation {
namespace plan {
void aGrabHandle(std::string params, bool* run) {
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

void aPose(std::string params, bool* run) {
  // Get Parameters
  int i_object = params.find("_");
  int i_hand = params.find("_", i_object + 1);
  string pose = params.substr(0, i_object);

  ROS_INFO("aPose - go to pose %s ", pose.c_str());
  
  bool result = robobreizh::callMovementServer("pose_" + robobreizh::toLower(pose));

  if(result)
    RoboBreizhManagerUtils::setPNPConditionStatus("PoseOK");
  else
    RoboBreizhManagerUtils::setPNPConditionStatus("PoseFailed");

  *run = 1;
}

void aCallMovementServer(std::string params, bool* run) {
  // Get Parameters
  string order = params;

  ROS_INFO("aCallMovementServer - send order %s to movement server", order.c_str());

  bool result = robobreizh::callMovementServer(order);

  if(result)
    RoboBreizhManagerUtils::setPNPConditionStatus("CallOK");
  else
    RoboBreizhManagerUtils::setPNPConditionStatus("CallFailed");

  *run = 1;
}

void aGraspObject(std::string params, bool* run) {
  // Get Parameters
  int i_object = params.find("_");
  int i_hand = params.find("_", i_object + 1);
  string object = params.substr(0, i_object);
  string hand = params.substr(i_object + 1, i_hand);  // Left, Right, Both

  ROS_INFO("aGraspObject - grasp object %s using %s hand", object.c_str(), hand.c_str());

  // CV - Find and localise handle
  // Manipulation - Grasp handle  => Both actions should be on separate modules, with one to obtain object position,
  // used in the following.
  // manipulation::generic::grabHandle(object, hand);

  typedef actionlib::SimpleActionClient<manipulation_pepper::MovementAction> Client;
  Client client("movement", true);
  client.waitForServer();
  manipulation_pepper::MovementGoal goal;
  std::vector<double> target = {0.0};

  if(hand=="Both"){
    goal.order = "grab_2arms";
    goal.target = target;
  }
  else if (hand=="Right" && object=="Bag"){
    goal.order = "grab_bag";
    goal.target = target;
  }
  else if (hand=="Right"){
    goal.order = "grab_2arms";
    goal.target = target;
  }
  else {
    ROS_WARN("Unsupported parameter %s", hand.c_str());
  }

  client.sendGoal(goal);
  client.waitForResult(ros::Duration(30.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Grasp Completed");
  else
    printf("Current State: %s\n", client.getState().toString().c_str());
    ROS_INFO("GRASP FAILED");

  RoboBreizhManagerUtils::setPNPConditionStatus("GraspOK");
  *run = 1;
}

void aPutObject(std::string params, bool* run) {
  // Get Parameters
  string hand = params;

  ROS_INFO("aPutObject - put object in %s hand", hand.c_str());  // Left, Right, Both

  // Manipulation - Put object held on a certain hand on a certain surface position
  if(hand == "Both")
    bool result = robobreizh::callMovementServer("release_grab_2arms");

  RoboBreizhManagerUtils::setPNPConditionStatus("PutOK");
  *run = 1;
}

void aPourObject(std::string params, bool* run) {
  // Get Parameters
  int i_hand = params.find("_");
  int i_target = params.find("_", i_hand + 1);
  string hand = params.substr(0, i_hand);  // Left, Right, Both
  string target = params.substr(i_hand + 1, i_target);

  ROS_INFO("aPourObject - pour object of %s hand to target %s", hand.c_str(), target.c_str());

  RoboBreizhManagerUtils::setPNPConditionStatus("PourOK");
  *run = 1;
}
void aDropObject(std::string params, bool* run) {
  // Get Parameters
  string hand = params;

  ROS_INFO("aDropObject - drop object in %s hand", hand.c_str());

  if(hand == "Both")
    bool result = robobreizh::callMovementServer("release_grab_2arms");
  else if(hand == "Right")
    ROS_WARN("TO DO");
  else if(hand == "Left")
    ROS_WARN("TO DO");
  else
    ROS_WARN("aDropObject - Unrecognized hand : %s", hand.c_str());

  // Manipulation - Put object held on a certain hand on a certain position
  manipulation::generic::dropObject(hand);
}

void aPullObject(std::string params, bool* run) {
  // Grasp and Pull object
  int i_hand = params.find("_");
  int i_target = params.find("_", i_hand + 1);
  string hand = params.substr(0, i_hand);  // Left, Right, Both
  string target = params.substr(i_hand + 1, i_target);

  ROS_INFO("aPullObject - Pull object %s using %s hand(s)", hand.c_str(), target.c_str());

  RoboBreizhManagerUtils::setPNPConditionStatus("PullOK");
  *run = 1;
}
void aBendArms(string params, bool* run) {
  if (params == "Right") {
    // bend the Right arm by 90 degree to hold the bag
    robobreizh::manipulation::generic::bendArm(params);

  } else if (params == "Left") {
    // bend the left arm by 90 degree to hold the bag
  } else {
    ROS_ERROR("aBendArms - Unknown arm %s", params.c_str());
  }
  // bend the arm by 90 degree to hold the bag
  RoboBreizhManagerUtils::setPNPConditionStatus("Done");
  *run = 1;
}
void aIsObjectCloseEnoughToGrasp(std::string params, bool* run) {
  int i_object = params.find("_");
  int i_distance = params.find("_", i_object + 1);
  string target_object = params.substr(0, i_object);
  float distance = std::stof(params.substr(i_object + 1, i_distance));

  ROS_INFO("[ aIsObjectCloseEnoughToGrasp ] - Checking if object %s is close enough to grasp (distance : %s)", target_object.c_str(), std::to_string(distance).c_str());
  
  // Retrieve object position from the database
  robobreizh::database::ObjectModel object_model;
  std::vector<robobreizh::database::Object> object_vec = object_model.getObjectByLabel(target_object);
  size_t size = object_vec.size();
  if (object_vec.size() != 0) {
    if (object_vec.size() == 1) {
      robobreizh::database::Object object = object_vec[0];
      ROS_INFO("[ aIsObjectCloseEnoughToGrasp ] - Found %s in the database", target_object.c_str());
      ROS_INFO("[ aIsObjectCloseEnoughToGrasp ] - Distance :  %f", object.distance);
      if(object.distance <= distance)
        RoboBreizhManagerUtils::setPNPConditionStatus("ObjectCloseEnough");
      else
        RoboBreizhManagerUtils::setPNPConditionStatus("ObjectTooFar");
    } else {
      ROS_WARN("[ aIsObjectCloseEnoughToGrasp ] - More than one %s found in the database using the latest", target_object.c_str());
      robobreizh::database::Object object = object_vec[size - 1];
      if(object.distance <= distance)
        RoboBreizhManagerUtils::setPNPConditionStatus("ObjectCloseEnough");
      else
        RoboBreizhManagerUtils::setPNPConditionStatus("ObjectTooFar");
    }
  } else {
    ROS_ERROR("[ aIsObjectCloseEnoughToGrasp ] - No object found in the database");
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
  }

  //RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);

  *run = 1;
}

void aStopMovement(std::string params, bool* run) {

  bool result = robobreizh::callMovementServer("stop");

  RoboBreizhManagerUtils::setPNPConditionStatus("MovementStopped");
  *run = 1;
}
}  // namespace plan
}  // namespace manipulation
}  // namespace robobreizh
