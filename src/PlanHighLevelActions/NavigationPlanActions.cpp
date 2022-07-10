#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "GenericActions/NavigationGenericActions.hpp"
#include "ManagerUtils.hpp"
#include "DatabaseModel/NavigationModel.hpp"
#include "DatabaseModel/GPSRActionsModel.hpp"
#include "SQLiteUtils.hpp"


using namespace std;

using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

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

    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
    *run = 1;
}

void aFollowHuman(std::string params, bool* run)
{
    // Navigation - Follow human
    ROS_INFO("aFollowHuman - Following human");
    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
    *run = 1;
}

void aMoveTowardsLocation(string params, bool* run)
{
    // Move towards a certain location, not an object position
    // Navigation - Move towards a specific place
    string location = params;

    if (params == "GPSR")
    {
        GPSRActionsModel gpsrActionsDb;
        location = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::destination);
    } else {
        location = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(params);
    }
        
    

    ROS_INFO("aMoveTowardsLocation - moving towards %s", location.c_str());

    robobreizh::NavigationPlace np;
    robobreizh::database::NavigationModel nm;
    np = nm.getLocationFromName(location);

    navigation::generic::moveTowardsPosition(np.pose, np.angle); 
    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);

    *run = 1;

}

void aMoveTowardsHuman(string params, bool* run)
{
    string humanName;
    if (params.empty())
    {
        ROS_INFO("aMoveTowardsHuman - moving towards any Human");
    }
    
    else
    {
        if (params == "GPSR")
        {
            GPSRActionsModel gpsrActionsDb;
            humanName = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
        }
        else
            humanName = params;
        ROS_INFO("aMoveTowardsHuman - Moving towards specific Human called %s", humanName.c_str());
    }
    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
}

void aMoveTowardsGPSRTarget(string params, bool* run)
{
    // Move towards a specific object, not a room location
    GPSRActionsModel gpsrActionsDb;
    string target_object = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item);

    ROS_INFO("aMoveTowardsGPSRTarget - Moving towards object %s", target_object.c_str());
    
    // Move towards target position
    //aMoveTowardsLocation(target, run);
}

void aRotate(string params, bool* run)
{
    // Parse action parameters from "commands" parameter (not implemented yet)
    std::cout << params << std::endl;
    float angle = 0.0;
    if (!params.empty()){
        angle = std::stod(params);
    }
    ROS_INFO("aRotate - turning %f", angle);
     
    navigation::generic::rotateOnPoint(angle); 
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
   *run = 1;
}

void aTurnTowards(string params, bool* run)
{
    // Parse action parameters from "commands" parameter (not implemented yet)
    string location = params;
    ROS_INFO("aTurnTowards - turning towards %s", location.c_str());
    
    // Move towards target
   *run = 1;
}
} // namespace plan
} // namespace navigation
} // namespace robobreizh
