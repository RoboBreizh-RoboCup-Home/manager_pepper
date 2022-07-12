#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <pnp_ros/names.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>

#include "SQLiteUtils.hpp"
#include "ManagerUtils.hpp"
#include "PlanHighLevelActions/DialogPlanActions.hpp"
#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"
#include "PlanHighLevelActions/ManipulationPlanActions.hpp"
#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "PlanHighLevelActions/OtherPlanActions.hpp"
#include "PlanHighLevelActions/VisionPlanActions.hpp"
#include "PlanHighLevelActions/GesturePlanActions.hpp"

namespace dialog = robobreizh::dialog::plan;
namespace initialisation = robobreizh::initialisation::plan;
namespace manipulation = robobreizh::manipulation::plan;
namespace navigation = robobreizh::navigation::plan;
namespace other = robobreizh::other::plan;
namespace vision = robobreizh::vision::plan;
namespace gesture = robobreizh::gesture::plan;


class RoboBreizhManager : public PNPActionServer
{
private:

    ros::NodeHandle handle;
    //ros::Publisher event_pub;

public:
    RoboBreizhManager() : PNPActionServer() { 

        std::string robotName = "RoboBreizh";
        handle.setParam("/robot_name", robotName);
        ROS_INFO("ROBOTNAME: %s", robotName.c_str());

        // Register parameters
        //event_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);

        // Register actions
        register_action("initCarryMyLuggage", &initialisation::aInitCarryMyLuggage);
        register_action("initGPSR", &initialisation::aInitGPSR);
        register_action("initReceptionist", &initialisation::aInitReceptionist);
        register_action("initFindMyMate", &initialisation::aInitFindMyMate);
        register_action("initRestaurant", &initialisation::aInitRestaurant);
        register_action("initFarewell", &initialisation::aInitFarewell);
        register_action("initStoringGroceries", &initialisation::aInitStoringGroceries);

        register_action("DialogSay", &dialog::aSay);
        register_action("DialogAskHumanPlaceLastObjectOnTablet", &dialog::aDialogAskHumanPlaceLastObjectOnTablet);
        register_action("DialogGreetHuman", &dialog::aGreetHuman);
        register_action("DialogAskHumanToStartTask", &dialog::aAskHumanToStartTask);
        register_action("DialogAskHumanRepeat", &dialog::aAskHumanRepeat);
        register_action("DialogAskHandOver", &dialog::aAskHandOverObject);
        register_action("DialogAskHumanToFollowToLocation", &dialog::aAskHumanToFollowToLocation);
        register_action("DialogTellOperatorReadyToGo", &dialog::aTellReadyToGo);
        register_action("DialogSayGoodbyeToGuest", &dialog::aTellReadyToGo);
        register_action("DialogListenOrders", &dialog::aListenOrders);
        register_action("DialogAskHumanToFollow", &dialog::aAskHumanToFollow);
        register_action("DialogTellHumanObjectLocation", &dialog::aTellHumanObjectLocation);
        register_action("DialogAskHumanTake", &dialog::aAskHumanTake);
        register_action("DialogAskActionConfirmation", &dialog::aAskActionConfirmation);
        register_action("DialogAskHuman", &dialog::aAskHuman);
        register_action("DialogListenConfirmation", &dialog::aListenConfirmation);
        register_action("DialogListen", &dialog::aListen);
        register_action("DialogIntroduceAtoB", &dialog::aIntroduceAtoB);
        register_action("DialogOfferSeatToHuman", &dialog::aOfferSeatToHuman);
        register_action("DialogDescribeHuman", &dialog::aDescribeHuman);
        register_action("DialogAskHumanNameConfirmation", &dialog::aAskHumanNameConfirmation);
        register_action("DialogTellHumanDestinationArrived", &dialog::aTellHumanDestinationArrived);
        register_action("DialogAskOperatorHelp", &dialog::aAskOperatorHelpOrder);
        register_action("DialogChitChat", &dialog::aDialogChitChat);

        /* register_action("VisionFindHumanFilter", &vision::aFindHumanFilter); */
        register_action("VisionWaitForOperator", &vision::aWaitForOperator);
        register_action("VisionFindObject", &vision::aFindObject);
        register_action("VisionFindHuman", &vision::aFindHuman);
        register_action("VisionWaitForDoorOpening", &vision::aWaitForDoorOpening);
        register_action("VisionFindEmptySeat", &vision::aFindEmptySeat);
        register_action("VisionFindHumanAndStoreFeatures", &vision::aFindHumanAndStoreFeatures);
        register_action("VisionFindHumanAndStoreFeaturesWithDistanceFilter", &vision::aFindHumanAndStoreFeaturesWithDistanceFilter);
        register_action("VisionWaitForHumanWavingHand", &vision::aWaitForHumanWavingHand);
        register_action("VisionLocatePositionToPlaceObject", &vision::aLocatePositionToPlaceObject);
        register_action("VisionFindCabDriver", &vision::aFindCabDriver);
        register_action("VisionFindObjectPointedByHuman", &vision::aFindObjectPointedByHuman);

        register_action("ManipulationGrabHandle", &manipulation::aGrabHandle);
        register_action("ManipulationDropObject", &manipulation::aDropObject);
        register_action("ManipulationLook", &manipulation::aLook);
        register_action("ManipulationPointAt", &manipulation::aPointAt);

        register_action("ManipulationBendArms", &manipulation::aBendArms);

        register_action("NavigationMoveTowardsObject", &navigation::aMoveTowardsObject);
        register_action("NavigationFollowHuman", &navigation::aFollowHuman);
        register_action("NavigationMoveTowardsLocation", &navigation::aMoveTowardsLocation);
        register_action("NavigationMoveTowardsHuman", &navigation::aMoveTowardsHuman);
        register_action("NavigationMoveTowardsGPSRTarget", &navigation::aMoveTowardsGPSRTarget);
        register_action("NavigationRotate", &navigation::aRotate);
        register_action("NavigationTurnTowards", &navigation::aTurnTowards);
        register_action("NavigationMoveBehindHuman", &navigation::aMoveBehindHuman);

        register_action("GestureLookAt", &gesture::aLookAt);

        register_action("ProcessOrders", &other::aGPSRProcessOrders);
        register_action("OtherCheckForMoreGuests", &other::aCheckForMoreGuests);
        register_action("OtherCheckForMoreObjectTofind", &other::aCheckForMoreObjectTofind);
        register_action("OtherChangePlan", &other::aChangePlan);
        register_action("OtherIsHumanKnown", &other::aIsHumanKnown);


        // Register conditions
        //register_condition("closeToHome",&closeToHomeCond);
    }
};

warehouse_ros_sqlite::DatabaseConnection* robobreizh::SQLiteUtils::conn_ = new warehouse_ros_sqlite::DatabaseConnection();
ros::Publisher* robobreizh::RoboBreizhManagerUtils::pnpPublisher_;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robobreizh_manager");

  // Initialise SQLite database
  robobreizh::SQLiteUtils::conn_->setParams(":memory:", 0);
  bool ret = robobreizh::SQLiteUtils::conn_->connect();

  // Prepare ROStopic publisher for PNP Status

  //robobreizh::RoboBreizhManagerUtils::rosHandle_ = new ros::NodeHandle();

  ros::NodeHandle rosHandle;
  //robobreizh::RoboBreizhManagerUtils::pnpPublisher_ = new ros::Publisher();
  //ros::Publisher rosPnpPublisher = *robobreizh::RoboBreizhManagerUtils::pnpPublisher_;
  ros::Publisher rosPnpPublisher = rosHandle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 1000);
  robobreizh::RoboBreizhManagerUtils::pnpPublisher_ = &rosPnpPublisher;

  RoboBreizhManager robobreizh_manager;
  robobreizh_manager.start();
  ros::spin();

  return 0;
}
