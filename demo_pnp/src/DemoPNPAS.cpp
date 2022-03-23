#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <pnp_ros/names.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>

#include "DemoActions.h"


class MyPNPActionServer : public PNPActionServer
{
private:

    ros::NodeHandle handle;

public:

    MyPNPActionServer() : PNPActionServer() { 

        // robotname external defined in MyActions.h/cpp
        handle.param("robot_name",robotname,std::string(""));
        ROS_INFO("ROBOTNAME: %s",robotname.c_str());

        register_action("init",&ainit);
        register_action("check_color",&acheck_color);

        register_condition("closeToHome",&closeToHomeCond);
	
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mypnpas");

  MyPNPActionServer mypnpas;
  mypnpas.start();
  ros::spin();

  return 0;
}
