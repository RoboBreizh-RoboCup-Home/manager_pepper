#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "GenericActions/NavigationGenericActions.hpp"

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
    float z = 0;

    // Navigation - Move towards a specific place
    string location = params;
    ROS_INFO("aMoveTowardsLocation - moving towards %s", location.c_str());

    // Dirty example for demonstration only => Positions needs to be located elsewhere
    if (location == "arena")
    {
        float x = 1;
        float y = 1;
        float z = 1;
    }

    navigation::generic::moveTowardsPosition(x, y, z);
}

void aMoveTowardsHuman(string params, bool* run)
{
    ROS_INFO("aMoveTowardsHuman - moving towards Human");
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
