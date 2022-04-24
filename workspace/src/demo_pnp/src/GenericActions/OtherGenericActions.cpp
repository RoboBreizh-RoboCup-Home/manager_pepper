#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/OtherGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace other
{
namespace generic
{
    bool waitForGoSignal()
    {
        ros::NodeHandle nh;
        boost::shared_ptr<std_msgs::String const> shared_msg;
        std_msgs::String msg;
        cout << "wait_for_go_signal - Waiting for go signal from /robobreizh/manager/go" << endl;

        shared_msg = ros::topic::waitForMessage<std_msgs::String>("/robobreizh/manager/go", nh);

        if (shared_msg != NULL)
        {
            msg = *shared_msg;
            std::cout << "waitForGoSignal - Let's go!" << msg.data << std::endl;
            return true;
        }
        else
        {
            std::cout << "waitForGoSignal - ERROR" << std::endl;
            return false;
        }
    }
} // namespace generic
} // namespace other
} // namespace robobreizh