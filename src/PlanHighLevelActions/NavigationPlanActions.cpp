#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "GenericActions/NavigationGenericActions.hpp"
#include "ManagerUtils.hpp"

using namespace std;

namespace robobreizh
{
namespace navigation
{
namespace plan
{

void aMoveTowardsObject(std::string params, bool* run)
{
    // Get Parameter(s)
    string object = params;
    ROS_INFO("aMoveTowardsObject - Currently moving torwards %s", object.c_str());

    // Navigation - Move towards a certain position
    navigation::generic::moveTowardsObject(object);
}

void aFollowHuman(std::string params, bool* run)
{
    // Navigation - Follow human
    ROS_INFO("aFollowHuman - Following human");
}

void aMoveTowardsLocation(string params, bool* run)
{
    float x = 0;
    float y = 0;
    float theta = 0.0;
    int time = 0;

    // Navigation - Move towards a specific place
    string location = params;
    ROS_INFO("aMoveTowardsLocation - moving towards %s", location.c_str());
    
    x = 0.5;
    y = 2.0;
    theta = 90.0;
    time = 20;

    bool destReached = false;
    do
    {
        destReached =  navigation::generic::moveTowardsPosition(x, y, theta, time); // TODO: Use Enum instead of bool (Open, closed, notfound)
    } while (!destReached); // TODO: Add timer for timeout
    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
    *run = 1;

}

void aMoveTowardsHuman(string params, bool* run)
{
    ROS_INFO("aMoveTowardsHuman - moving towards Human");
    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
}

void aMoveTowardsGPSRTarget(string params, bool* run)
{
    // Parse action parameters from "commands" parameter (not implemented yet)
    string target = "Undefined";
    
    // Move towards target
    aMoveTowardsLocation(target, run);
}

} // namespace plan
} // namespace navigation
} // namespace robobreizh
