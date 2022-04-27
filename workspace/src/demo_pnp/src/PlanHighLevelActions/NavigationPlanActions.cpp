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
    string object = params;
    ROS_INFO("aMoveTowardsObject - Currently moving torwards %s", object.c_str());
    robobreizh::navigation::generic::moveTowardsObject(object);
}

void aFollowHuman(std::string params, bool* run)
{
    ROS_INFO("aFollowHuman - Following human");
}
} // namespace plan
} // namespace navigation
} // namespace robobreizh
