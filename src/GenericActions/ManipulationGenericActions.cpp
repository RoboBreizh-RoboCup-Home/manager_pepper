#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/ManipulationGenericActions.hpp"

using namespace std;

namespace robobreizh
{
    namespace manipulation
    {
        namespace generic
        {

            bool grabHandle(string object, string hand)
            {
                //* Instead on object in string type, may be more logical to use position instead *//
                return true;
            }
            bool dropObject(string hand)
            {
                //* A variant with the position where we need to put the object may be pretty useful IMHO *//
                return true;
            }

            bool lookUp()
            {
                ros::NodeHandle nh;
                ros::ServiceClient client = nh.serviceClient<manipulation_pepper::EmptySrv>("/robobreizh/manipulation_pepper/look_up");
                perception_pepper::object_detection_service srv;

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
                ros::ServiceClient client = nh.serviceClient<manipulation_pepper::EmptySrv>("/robobreizh/manipulation_pepper/look_down");
                perception_pepper::object_detection_service srv;

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
                ros::ServiceClient client = nh.serviceClient<manipulation_pepper::EmptySrv>("/robobreizh/manipulation_pepper/look_around");
                perception_pepper::object_detection_service srv;

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
                ros::ServiceClient client = nh.serviceClient<manipulation_pepper::EmptySrv>("/robobreizh/manipulation_pepper/point_in_front");
                perception_pepper::object_detection_service srv;

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
        } // namespace generic
    }     // namespace manipulation
} // namespace robobreizh