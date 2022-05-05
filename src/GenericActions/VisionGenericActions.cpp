#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/VisionGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace vision
{
namespace generic
{
    bool waitForHuman()
    {
        return true;
        // Wait for vision module to find and locate a Human approaching the robot
        // Example of code commented (depending on the type you want to use)

        /*ros::NodeHandle nh;
        boost::shared_ptr<std_msgs::String const> shared_msg;
        std_msgs::String msg;
        ROS_INFO("wait_for_go_signal - Waiting for go signal from /robobreizh/manager/go");

        shared_msg = ros::topic::waitForMessage<std_msgs::String>("/robobreizh/manager/go", nh);

        if (shared_msg != NULL)
        {
            msg = *shared_msg;
            ROS_INFO("waitForHuman - Human found in front of the robot! %s", msg.data.c_str());
            return true;
        }
        else
        {
            ROS_INFO("waitForGoSignal - ERROR");
            return false;
        }*/
    }

    bool findObject(std::string objectName)
    {
        // bool is probably not the right output type, a position seems more relevant
        return true;
    }

    bool isDoorOpened() // TODO: What if door not found => Use Enum instead (Open, closed, NotFound)
    {
        ros::NodeHandle nh;
        boost::shared_ptr<std_msgs::Float32 const> shared_msg;
        std_msgs::Float32 msg;
        ROS_INFO("wait_for_go_signal - Waiting for go signal from /robobreizh/perception_pepper/door_detection/open");

        shared_msg = ros::topic::waitForMessage<std_msgs::Float32>("/robobreizh/perception_pepper/door_detection/open", nh);

        if (shared_msg != NULL)
        {
            msg = *shared_msg;
            float test;
            test = 2.1;
            ROS_INFO("Door opened at distance ++ %f", test);
            ROS_INFO("Door opened at distance  %f", msg.data);

            return true;
        }
        else
        {
            ROS_INFO("waitForDoorSignal - ERROR");
            return false;
        }
    }

} // namespace generic
} // namespace vision
} // namespace robobreizh