#include <std_msgs/String.h>
#include <ros/ros.h>
#include <pnp_ros/names.h>
#include "PlanHighLevelActions/OtherPlanActions.hpp"




using namespace std;

namespace robobreizh
{
namespace other
{
namespace plan
{
    void aGPSRProcessOrders(string params, bool* run)
    {
        ros::NodeHandle handle;
        ros::Publisher pnp_condition_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);
        std_msgs::String cond;

        cond.data = "nextOrderGo";
        pnp_condition_pub.publish(cond);
        *run = 1;
    }
} // namespace plan
} // namespace other
}// namespace robobreizh