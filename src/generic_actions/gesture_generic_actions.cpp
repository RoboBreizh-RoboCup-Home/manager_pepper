#include <ros/ros.h>
#include <std_msgs/String.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/GestureGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace gesture
{
namespace generic
{

bool lookUp()
{
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<manipulation_pepper::EmptySrv>("/robobreizh/manipulation_pepper/look_up");
  manipulation_pepper::EmptySrv srv;

  if (client.call(srv))
  {
    ROS_INFO("Call to looking up OK");
  }
  else
  {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}
bool lookDown()
{
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<manipulation_pepper::EmptySrv>("/robobreizh/manipulation_pepper/look_down");
  manipulation_pepper::EmptySrv srv;

  if (client.call(srv))
  {
    ROS_INFO("Call to looking up OK");
  }
  else
  {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}
bool lookAround()
{
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<manipulation_pepper::EmptySrv>("/robobreizh/manipulation_pepper/look_around");
  manipulation_pepper::EmptySrv srv;

  if (client.call(srv))
  {
    ROS_INFO("Call to looking up OK");
  }
  else
  {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}

bool pointInFront()
{
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<manipulation_pepper::EmptySrv>("/robobreizh/manipulation_pepper/point_in_front");
  manipulation_pepper::EmptySrv srv;

  if (client.call(srv))
  {
    ROS_INFO("Call to looking up OK");
  }
  else
  {
    ROS_INFO("look up service - ERROR");
    return false;
  }
  return true;
}
}  // namespace generic
}  // namespace gesture
}  // namespace robobreizh