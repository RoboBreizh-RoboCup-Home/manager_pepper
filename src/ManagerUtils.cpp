#include <std_msgs/String.h>
#include <ros/ros.h>
#include <pnp_ros/names.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp> 

#include "ManagerUtils.hpp"

using namespace std;

namespace robobreizh
{
    string RoboBreizhManagerUtils::getPNPConditionStatus()
    {
        // TODO: Add try catch if failure
        ros::NodeHandle nh;
        boost::shared_ptr<std_msgs::String const> shared_msg;
        std_msgs::String msg;

        shared_msg = ros::topic::waitForMessage<std_msgs::String>(TOPIC_PNPCONDITION, nh);

        if (shared_msg != NULL)
        {
            return msg.data;
        }
    }

    bool RoboBreizhManagerUtils::setPNPConditionStatus(string status)
    {
        // TODO: Add Try catch if failure

        ros::NodeHandle handle;
        ros::Publisher pnp_condition_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);
        std_msgs::String cond;

        cond.data = status;
        pnp_condition_pub.publish(cond);

        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000)); // TODO: Please remove ASAP this horrendous instruction :/

        return true;
    }
}// namespace robobreizh