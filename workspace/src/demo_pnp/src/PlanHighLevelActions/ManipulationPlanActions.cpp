#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/ManipulationPlanActions.hpp"

using namespace std;

namespace robobreizh
{
namespace manipulation
{
namespace plan
{

void aGrabHandle(std::string params, bool* run)
{
    int i_object=params.find("_");
    int i_hand=params.find("_", i_object + 1);
    string object=params.substr(0, i_object);
    string hand=params.substr(i_object + 1, i_hand);

    ROS_INFO("aGrabHandle - grab object %s using %s hand", object.c_str(), hand.c_str());

    // Add generic function used for this purpose
}

void aDropObject(std::string params, bool* run)
{
    string hand = params;

    ROS_INFO("aDropObject - drop object in %s hand", hand.c_str());

    // TODO: Add Generic function
}

} // namespace plan
} // namespace manipulation
}// namespace robobreizh
