#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <navigation_pep/NavigationDestination.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Pose.h"

//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/NavigationGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace navigation
{
namespace generic
{
bool moveTowardsObject(string objectName /** Or object position if you prefer**/)
{
    // We can for example use a ros service here
    return true;
}

bool convertThetaToQuat(float theta){ 
    /*tf2::Quaternion myQuaternion;

    myQuaternion.setRPY(0.0, 0.0, theta);

    myQuaternion.normalize();

    return myQuaternion;*/
    return true;
}

bool moveTowardsPosition(float x, float y, float theta, int time)
{
    /*ros::NodeHandle nh;
    geometry_msgs::Pose msg;
    tf2::Quaternion orientation;

    orientation.setRPY(0.0, 0.0, theta);

    orientation.normalize();

    tf2::convert(orientation, msg.orientation);
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = 0.0;
    
    ROS_INFO("Sending goal - x: %f y: %f theta: %f", x, y, theta);
    ROS_INFO("Sending goal ROS mode - x: %f y: %f ", msg.position.x, msg.position.y);

    ros::ServiceClient client = nh.serviceClient<navigation_pep::NavigationDestination>("/robobreizh/navigation_pepper/move_to_goal");
    navigation_pep::NavigationDestination srv;
    srv.request.pose = msg; 
    srv.request.timer = time;
    
    if (client.call(srv))
    {
        ROS_INFO("Navigation success: %s", srv.response.success);
    }
    else{
        ROS_ERROR("Failed to call service move_to_goal");
        return false;      
    }*/

    return true;
}

} // namespace generic
} // namespace navigation
} // namespace robobreizh