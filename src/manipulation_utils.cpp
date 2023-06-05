#include <std_msgs/String.h>
#include <ros/ros.h>

#include <manipulation_pepper/MovementAction.h>
#include <actionlib/client/simple_action_client.h>

#include "manipulation_utils.hpp"

using namespace std;

namespace robobreizh {
  bool callMovementServer(const string& order) {
    typedef actionlib::SimpleActionClient<manipulation_pepper::MovementAction> Client;
    Client client("movement", true);
    client.waitForServer();

    manipulation_pepper::MovementGoal goal;
    vector<double> target = {0.0};

    goal.order = order;
    goal.target = target;

    client.sendGoal(goal);
    client.waitForResult(ros::Duration(15.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Order %s Completed", order.c_str());
      return true;
    }   
    else{
      printf("Current State: %s\n", client.getState().toString().c_str());
      ROS_INFO("Order %s Failed", order.c_str());
      return false;
    }
  }
}