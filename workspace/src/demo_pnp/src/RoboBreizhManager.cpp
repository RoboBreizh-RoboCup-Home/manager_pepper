#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <pnp_ros/names.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>

#include "PlanHighLevelActions/DialogPlanActions.hpp"
#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"
#include "PlanHighLevelActions/ManipulationPlanActions.hpp"
#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "PlanHighLevelActions/OtherPlanActions.hpp"
#include "PlanHighLevelActions/VisionPlanActions.hpp"

namespace dialog = robobreizh::dialog::plan;
namespace initialisation = robobreizh::initialisation::plan;
namespace manipulation = robobreizh::manipulation::plan;
namespace navigation = robobreizh::navigation::plan;
namespace other = robobreizh::other::plan;
namespace vision = robobreizh::vision::plan;


class RoboBreizhManager : public PNPActionServer
{
private:

    ros::NodeHandle handle;

public:
    RoboBreizhManager() : PNPActionServer() { 

        std::string robotName = "Roy Batty";
        handle.param("robot_name", robotName, std::string(""));
        ROS_INFO("ROBOTNAME: %s", robotName.c_str());

        register_action("initCarryMyLuggage", &initialisation::aInitCarryMyLuggage);
        //register_action("waitForGoSignal", &await_for_go_signal);
        //register_action("greetings", &agreetings);
        //register_action("waitForHuman", &await_for_human);
        //register_condition("closeToHome",&closeToHomeCond);
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mypnpas");

  RoboBreizhManager robobreizh_manager;
  robobreizh_manager.start();
  ros::spin();

  return 0;
}
