#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"
#include "ManagerUtils.hpp"

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

void aInitGPSR(string params, bool* run)
{
    // TODO: Add global variables initiailisation here
    ROS_INFO("1.5 General Purpose Service Robot - initialisation");
    RoboBreizhManagerUtils::setPNPConditionStatus("GPSR_Init_done");
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
