#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sqlite3.h>
#include <warehouse_ros_sqlite/database_connection.h>
#include <warehouse_ros_sqlite/utils.h>

#include <pnp_ros/names.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>

#include "sqlite_utils.hpp"
#include "manager_utils.hpp"
#include "plan_high_level_actions/dialog_plan_actions.hpp"
#include "plan_high_level_actions/initialisation_plan_actions.hpp"
#include "plan_high_level_actions/manipulation_plan_actions.hpp"
#include "plan_high_level_actions/navigation_plan_actions.hpp"
#include "plan_high_level_actions/other_plan_actions.hpp"
#include "plan_high_level_actions/vision_plan_actions.hpp"
#include "plan_high_level_actions/gesture_plan_actions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace dialog = robobreizh::dialog::plan;
namespace initialisation = robobreizh::initialisation::plan;
namespace manipulation = robobreizh::manipulation::plan;
namespace navigation = robobreizh::navigation::plan;
namespace other = robobreizh::other::plan;
namespace vision = robobreizh::vision::plan;
namespace gesture = robobreizh::gesture::plan;

/**
 * @brief RoboBreizhManager initializes PNPActionServer with symbolic symbols to High Level Actions functions.
 *
 */
class RoboBreizhManager : public PNPActionServer {
private:
  ros::NodeHandle handle;
  // ros::Publisher event_pub;

public:
  RoboBreizhManager() : PNPActionServer() {
    std::string robotName = "RoboBreizh";
    handle.setParam("/robot_name", robotName);
    ROS_INFO("ROBOTNAME: %s", robotName.c_str());

    // Register actions
    register_action("initCarryMyLuggage", &initialisation::aInitCarryMyLuggage);
    register_action("initGPSR", &initialisation::aInitGPSR);
    register_action("initReceptionist", &initialisation::aInitReceptionist);
#ifdef LEGACY
    register_action("initFindMyMate", &initialisation::aInitFindMyMate);
    register_action("initFarewell", &initialisation::aInitFarewell);
#endif
    register_action("initRestaurant", &initialisation::aInitRestaurant);
    register_action("initStoringGroceries", &initialisation::aInitStoringGroceries);
    register_action("initStickler", &initialisation::aInitStickler);
    register_action("initServeBreakfast", &initialisation::aInitServeBreakfast);
    register_action("initCleanTheTable", &initialisation::aInitCleanTheTable);

    register_action("DialogSay", &dialog::aSay);
    register_action("DialogGreet", &dialog::aGreet);
    register_action("DialogAskHumanTakeLastObject", &dialog::aDialogAskHumanTakeLastObject);
    register_action("DialogAskHumanToStartTask", &dialog::aAskHumanToStartTask);
    register_action("DialogAskHumanRepeat", &dialog::aAskHumanRepeat);
    register_action("DialogAskHumanToFollowToLocation", &dialog::aAskHumanToFollowToLocation);
    register_action("DialogTellHumanArriveDestination", &dialog::aTellHumanArriveAtDes);
    register_action("DialogListenOrders", &dialog::aListenOrders);
    register_action("DialogAskHumanToFollow", &dialog::aAskHumanToFollow);
    register_action("DialogTellHumanObjectLocation", &dialog::aTellHumanObjectLocation);
    register_action("DialogAskHumanTake", &dialog::aAskHumanTake);
    register_action("DialogAskActionConfirmation", &dialog::aAskActionConfirmation);
    register_action("DialogAskHuman", &dialog::aAskHuman);
    register_action("DialogListenConfirmation", &dialog::aListenConfirmation);
    register_action("DialogListen", &dialog::aListen);
    register_action("DialogDescribeHuman", &dialog::aDescribeHuman);
    register_action("DialogAskHumanNameConfirmation", &dialog::aAskHumanNameConfirmation);
    register_action("DialogTellHumanDestinationArrived", &dialog::aTellHumanDestinationArrived);
    register_action("DialogAskOperatorHelp", &dialog::aAskOperatorHelpOrder);
    register_action("DialogIntroduceAtoB", &dialog::aIntroduceAtoB);
    register_action("DialogOfferSeatToHuman", &dialog::aOfferSeatToHuman);
#ifdef LEGACY
    register_action("DialogChitChat", &dialog::aDialogChitChat);
#endif
    register_action("DialogAskHumanPlaceLastObjectOnTablet", &dialog::aDialogAskHumanPlaceLastObjectOnTablet);

    /* register_action("VisionFindHumanFilter", &vision::aFindHumanFilter); */
    register_action("VisionWaitForOperator", &vision::aWaitForOperator);
    register_action("VisionCheckNumOfDetection", &vision::aCheckNumsOfDetectionTime);
    register_action("VisionFindObject", &vision::aFindObject);
    register_action("VisionFindHuman", &vision::aFindHuman);
    register_action("VisionFindHumanWithTimeout", &vision::aFindHumanWithTimeout);
    register_action("VisionWaitForDoorOpening", &vision::aWaitForDoorOpening);
    register_action("VisionFindEmptySeat", &vision::aFindEmptySeat);
    register_action("VisionFindHumanAndStoreFeatures", &vision::aFindHumanAndStoreFeatures);
    register_action("VisionFindHumanAndStoreFeaturesWithDistanceFilter",
                    &vision::aFindHumanAndStoreFeaturesWithDistanceFilter);
    register_action("VisionLocatePositionToPlaceObject", &vision::aLocatePositionToPlaceObject);
#ifdef LEGACY
    register_action("VisionFindCabDriver", &vision::aFindCabDriver);
#endif
    register_action("VisionFindObjectPointedByHuman", &vision::aFindObjectPointedByHuman);
    register_action("VisionWaitForHumanWavingHand", &vision::aWaitForHumanWavingHand);
    register_action("VisionFindPersonWithShoes", &vision::aFindPersonWithShoes);
    register_action("VisionFindPersonWithoutDrink", &vision::aFindPersonWithoutDrink);
    register_action("VisionFindPersonLittering", &vision::aFindPersonLittering);
    register_action("VisionFindPersonForbiddenRoom", &vision::aFindPersonForbiddenRoom);
    register_action("VisionFindStickler", &vision::aFindStickler);

    register_action("ManipulationGrabHandle", &manipulation::aGrabHandle);
    register_action("ManipulationDropObject", &manipulation::aDropObject);
    register_action("ManipulationGraspObject", &manipulation::aGraspObject);
    register_action("ManipulationPutObject", &manipulation::aPutObject);
    register_action("ManipulationPourObject", &manipulation::aPourObject);
    register_action("ManipulationPullObject", &manipulation::aPullObject);
    register_action("ManipulationPose", &manipulation::aPose);
    register_action("ManipulationCallMovementServer", &manipulation::aCallMovementServer);
    register_action("ManipulationIsObjectCloseEnoughToGrasp", &manipulation::aIsObjectCloseEnoughToGrasp);
    register_action("ManipulationStopMovement", &manipulation::aStopMovement);
    register_action("ManipulationMoveArm", &manipulation::aMoveArm);
    register_action("ManipulationSetHand", &manipulation::aSetHand);

    register_action("NavigationMoveTowardsObject", &navigation::aMoveTowardsObject);
    register_action("NavigationFollowHuman", &navigation::aFollowHuman);
    register_action("NavigationMoveTowardsLocation", &navigation::aMoveTowardsLocation);
    register_action("NavigationMoveTowardsHuman", &navigation::aMoveTowardsHuman);
    register_action("NavigationMoveTowardsGPSRTarget", &navigation::aMoveTowardsGPSRTarget);
    register_action("NavigationRotate", &navigation::aRotate);
    register_action("NavigationTurnTowards", &navigation::aTurnTowards);
    register_action("NavigationMoveBehindHuman", &navigation::aMoveBehindHuman);
    // register_action("NavigationMoveStraight", &navigation::aMoveStraight);

    register_action("GestureLookAt", &gesture::aLookAt);
    register_action("ManipulationLook", &gesture::aLook);
    register_action("ManipulationPointAt", &gesture::aPointAt);
    register_action("ManipulationPointObjects", &gesture::aPointObject);
    register_action("ManipulationBendArms", &manipulation::aBendArms);

    register_action("CheckObjectAndHumanInDestination", &other::aCheckObjectAndHuman);
    register_action("ProcessOrders", &other::aGPSRProcessOrders);
    register_action("OtherCheckForMoreGuests", &other::aCheckForMoreGuests);
    register_action("OtherCheckMoreObjectToFind", &other::aCheckForMoreObjectTofind);
    register_action("OtherChangePlan", &other::aChangePlan);
    register_action("OtherIsHumanKnown", &other::aIsHumanKnown);
    register_action("OtherChooseTake", &other::aChooseTake);
    register_action("OtherChooseFind", &other::aChooseFind);
    register_action("OtherWait", &other::aWait);
    register_action("OtherSticklerUpdateFinished", &other::aSticklerUpdateFinished);
    // Register conditions
    // register_condition("closeToHome",&closeToHomeCond);
  }
};

// Set connection to sqlite file in order to have persistent storage
warehouse_ros_sqlite::DatabaseConnection* robobreizh::SQLiteUtils::conn_ =
    new warehouse_ros_sqlite::DatabaseConnection();
ros::Publisher* robobreizh::RoboBreizhManagerUtils::pnpPublisher_;

/**
 * @brief Starts robobreizh_manager node, initialize the database and start PNP plan listener
 * @pre The database must be initialized before starting the node
 * @pre The naoqi driver has to be started beforehand
 */

int main(int argc, char** argv) {
  ros::init(argc, argv, "robobreizh_manager");

  // Connect SQLite database
  std::string db_file_path = "/home/nao/robobreizh_pepper_ws/src/manager_pepper/manager_db/roboBreizhDb.db";
  robobreizh::SQLiteUtils::conn_->setParams(db_file_path, 0);
  bool ret = robobreizh::SQLiteUtils::conn_->connect();

  // Create ros node
  ros::NodeHandle rosHandle;
  ros::Publisher rosPnpPublisher = rosHandle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 1000);
  robobreizh::RoboBreizhManagerUtils::pnpPublisher_ = &rosPnpPublisher;

  // start plan listener
  RoboBreizhManager robobreizh_manager;
  robobreizh_manager.start();
  ros::spin();

  return 0;
}