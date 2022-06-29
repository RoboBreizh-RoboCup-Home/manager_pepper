#include <ros/ros.h>
#include <std_msgs/String.h>
#include <navigation_pep/NavigationDestination.h>
#include <navigation_pep/AngleSrv.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

            bool convertThetaToQuat(float theta)
            {
                tf2::Quaternion myQuaternion;

                myQuaternion.setRPY(0.0, 0.0, theta);

                myQuaternion.normalize();

                return myQuaternion;
            }

            bool moveTowardsPosition(geometry_msgs::Pose p, float angle)
            {
                ros::NodeHandle nh;

                tf2::Quaternion orientation;
                orientation.setRPY(0.0, 0.0, angle);
                orientation.normalize();
                tf2::convert(orientation, p.orientation);

                ROS_INFO("Sending goal ROS Position(%f,%f,%f), Orientation(%f,%f,%f,%f)", p.position.x, p.position.y, p.position.z, p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);

                ros::ServiceClient client = nh.serviceClient<navigation_pep::NavigationDestination>("/robobreizh/navigation_pepper/move_to_goal");
                navigation_pep::NavigationDestination srv;
                srv.request.pose = p;

                if (client.call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO("Navigation success: Goal achieved");
                    }
                    else
                    {
                        ROS_ERROR("Navigation timed out");
                        return false;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service move_to_goal");
                    return false;
                }
                return true;
            }

            bool rotateOnPoint(float angle)
            {
                ros::NodeHandle nh;

                ros::ServiceClient client = nh.serviceClient<navigation_pep::AngleSrv>("/robobreizh/navigation_pepper/rotate_on_point");
                navigation_pep::AngleSrv srv;
                srv.request.angle = angle;

                if (client.call(srv))
                {
                    ROS_INFO("Rotation done");
                }
                else
                {
                    ROS_ERROR("Failed to call service rotation_on_point");
                    return false;
                }
                return true;
            }

        } // namespace generic
    }     // namespace navigation
} // namespace robobreizh
