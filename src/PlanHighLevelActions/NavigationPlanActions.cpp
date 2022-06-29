#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "GenericActions/NavigationGenericActions.hpp"
#include "ManagerUtils.hpp"
#include "DatabaseModel/NavigationModel.hpp"
#include "DatabaseModel/GPSRActionsModel.hpp"
#include "SQLiteUtils.hpp"


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
    // Move towards a certain location, not an object position
    // Navigation - Move towards a specific place
    string location = params;

    if (params == "GPSR")
    {
        database::GPSRActionsModel gpsrActionsDb;
        std_msgs::Int32 current_action_id_int32;
        bool is_value_available = SQLiteUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

        database::GPSRAction gpsrAction = gpsrActionsDb.getAction(current_action_id_int32.data);
        location = gpsrAction.destination;
    }
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
    // Move towards a specific object, not a room location
    string target_object = "Undefined";
    database::GPSRActionsModel gpsrActionsDb;
    std_msgs::Int32 current_action_id_int32;
    bool is_value_available = SQLiteUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

    database::GPSRAction gpsrAction = gpsrActionsDb.getAction(current_action_id_int32.data);
    target_object = gpsrAction.object_item;

    ROS_INFO("aMoveTowardsGPSRTarget - Moving towards object %s", target_object.c_str());
    
    // Move towards target position
    //aMoveTowardsLocation(target, run);
}

void aRotate(string params, bool* run)
{
    // Parse action parameters from "commands" parameter (not implemented yet)
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
