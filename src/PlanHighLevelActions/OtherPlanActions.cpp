#include <std_msgs/String.h>
#include <ros/ros.h>
#include <pnp_ros/names.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp> 

#include "PlanHighLevelActions/OtherPlanActions.hpp"
#include "ManagerUtils.hpp"



using namespace std;

namespace robobreizh
{
namespace other
{
namespace plan
{
    void aGPSRProcessOrders(string params, bool* run)
    {
        RoboBreizhManagerUtils::setPNPConditionStatus("nextOrderGiveMeCup");
        *run = 1;
    }
} // namespace plan
} // namespace other
}// namespace robobreizh