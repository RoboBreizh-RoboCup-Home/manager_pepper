#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "GenericActions/NavigationGenericActions.hpp"
#include "ManagerUtils.hpp"
#include "DatabaseModel/NavigationModel.hpp"

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

    // Navigation - Move towards a specific place
    string location = params;
    ROS_INFO("aMoveTowardsLocation - moving towards %s", location.c_str());

    robobreizh::NavigationPlace np;
    robobreizh::database::NavigationModel nm;
    np = nm.getLocationFromName(location);

    navigation::generic::moveTowardsPosition(np.pose, np.angle); 
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");

    *run = 1;

}

void aMoveTowardsHuman(string params, bool* run)
{
    if (params.empty())
    {
        ROS_INFO("aMoveTowardsHuman - moving towards any Human");
    }
    
    else
    {
        ROS_INFO("aMoveTowardsHuman - Moving towards specific Human called %s", params.c_str());
    }
    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
}

void aMoveTowardsGPSRTarget(string params, bool* run)
{
    // Parse action parameters from "commands" parameter (not implemented yet)
    string target = "Undefined";
    
    // Move towards target
    aMoveTowardsLocation(target, run);
}

void aRotate(string params, bool* run)
{
    // Parse action parameters from "commands" parameter (not implemented yet)
    float angle = 0.0;
    if !params.empty(){
        angle = std::stod(params);
    }
    ROS_INFO("aTurnTowards - turning %f", angle);
     
    navigation::generic::rotateOnPoint(angle); 
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
   *run = 1;
}

} // namespace plan
} // namespace navigation
} // namespace robobreizh
