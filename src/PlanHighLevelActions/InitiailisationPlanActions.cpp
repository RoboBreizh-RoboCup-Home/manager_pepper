#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"

using namespace std;

namespace robobreizh
{
namespace initialisation
{
namespace plan
{

void aInitCarryMyLuggage (string params, bool* run)
{
    ROS_INFO("1.1 Carry My Luggage - initialisation done");
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
