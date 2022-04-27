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
        handle.setParam("/robot_name", robotName);
        ROS_INFO("ROBOTNAME: %s", robotName.c_str());


        register_action("initCarryMyLuggage", &initialisation::aInitCarryMyLuggage);

        register_action("DialogGreetHuman", &dialog::aGreetHuman);
        register_action("DialogAskHandOver", &dialog::aAskHandOverObject);
        register_action("DialogTellOperatorReadyToGo", &dialog::aTellReadyToGo);
        register_action("DialogSayGoodbyeToGuest", &dialog::aTellReadyToGo);

        register_action("VisionWaitForOperator", &vision::aWaitForOperator);
        register_action("VisionFindObject", &vision::aFindObject);

        register_action("ManipulationGrabHandle", &manipulation::aGrabHandle);
        register_action("ManipulationDropObject", &manipulation::aDropObject);

        register_action("NavigationMoveTowardsObject", &navigation::aMoveTowardsObject);
        register_action("NavigationFollowHuman", &navigation::aFollowHuman);


        //register_condition("closeToHome",&closeToHomeCond);
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robobreizh_manager");

  RoboBreizhManager robobreizh_manager;
  robobreizh_manager.start();
  ros::spin();

  return 0;
}
