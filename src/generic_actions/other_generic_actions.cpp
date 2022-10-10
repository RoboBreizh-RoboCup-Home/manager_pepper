#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>
#include <vector>
#include <string>
#include <boost/thread/thread.hpp>

#include "generic_actions/other_generic_actions.hpp"

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
  ROS_INFO("wait_for_go_signal - Waiting for go signal from /robobreizh/manager/go");

  shared_msg = ros::topic::waitForMessage<std_msgs::String>("/robobreizh/manager/go", nh);

  if (shared_msg != NULL)
  {
    msg = *shared_msg;
    ROS_INFO("waitForGoSignal - Let's go! %s", msg.data.c_str());
    return true;
  }
  else
  {
    ROS_INFO("waitForGoSignal - ERROR");
    return false;
  }
}

}  // namespace generic
}  // namespace other
}  // namespace robobreizh
