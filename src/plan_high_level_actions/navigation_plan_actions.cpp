#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

#include "plan_high_level_actions/navigation_plan_actions.hpp"
#include "plan_high_level_actions/dialog_plan_actions.hpp"
#include "generic_actions/navigation_generic_actions.hpp"
#include "generic_actions/vision_generic_actions.hpp"
#include "generic_actions/dialog_generic_actions.hpp"
#include "manager_utils.hpp"
#include "vision_utils.hpp"
#include "database_model/location_model.hpp"
#include "database_model/person_model.hpp"
#include "database_model/object_model.hpp"
#include "database_model/gpsr_actions_model.hpp"
#include "sqlite_utils.hpp"

#include "geometry_msgs/Twist.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

namespace robobreizh {
namespace navigation {
namespace plan {
void aMoveTowardsObject(std::string params, bool* run) {
  params = robobreizh::toLower(params);
  // Get Parameter(s)
  ROS_INFO("[ aMoveTowardsObject ] - Currently moving torwards object : %s", params.c_str());
  // Retrieve object position from the database
  robobreizh::database::ObjectModel object_model;
  std::vector<robobreizh::database::Object> object_vec = object_model.getObjectByLabel(params);
  size_t size = object_vec.size();
  if (object_vec.size() != 0) {
    if (object_vec.size() == 1) {
      robobreizh::database::Object object = object_vec[0];
      ROS_INFO("[ aMoveTowardsObject ] - Found %s in the database", params.c_str());
      ROS_INFO("[ aMoveTowardsObject ] - Moving towards bag");
      navigation::generic::moveTowardsPosition(object.position, 0.0);
      RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
    } else {
      ROS_WARN("[ aMoveTowardsObject ] - More than one %s found in the database using the latest", params.c_str());
      robobreizh::database::Object object = object_vec[size - 1];
      navigation::generic::moveTowardsPosition(object.position, 0.0);
      RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
    }
  } else {
    ROS_ERROR("[ aMoveTowardsObject ] - No bag found in the database");
    RoboBreizhManagerUtils::setPNPConditionStatus("NavItemNotFound");
  }

  *run = 1;
}

void aFollowHuman(std::string params, bool* run) {
  // Navigation - Follow human
  ROS_INFO("aFollowHuman - Following human");

  // // while a goal is not cancelled execute the code
  // do {
  //   // call the perception to retrieve the person position
  //   geometry_msgs::Pose tracker_pose = vision::generic::getTrackerPersonPose();
  //   // set a goal to that person position
  //   // navigation::generic::moveTowardsPosition(tracker_pose);
  //   geometry_msgs::PoseWithCovariance robot_pose = navigation::generic::getCurrentPosition();
  //   float angle =
  //       std::atan2(person.position.y - robot_pose.pose.position.y, person.position.x - robot_pose.pose.position.x);
  //   navigation::generic::moveTowardsPosition(person.position, angle);
  // } while (navigation::generic::isMoveBaseGoal());
  // RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");

  ROS_INFO("aMoveTowardsHuman - Moving towards Human ");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("/robobreizh/start_tracker", 1000);
  std_msgs::String msg;

  std::stringstream ss;
  ss << "start";
  msg.data = ss.str();
  chatter_pub.publish(msg);

  *run = 1;
}

// void aFollowHuman(std::string params, bool* run) {
//   // Navigation - Follow human
//   ROS_INFO("aFollowHuman - Following human");

//   //--------------------------------------------------------------
//   database::PersonModel pm;
//   database::Person person = pm.getLastPerson();  // pm.getPersonByName(human_name);
//   //--------------------------------------------------------------
//   // gpsr_gesture_detection

//   geometry_msgs::PoseWithCovariance robot_pose = navigation::generic::getCurrentPosition();
//   float angle =
//       std::atan2(person.position.y - robot_pose.pose.position.y, person.position.x - robot_pose.pose.position.x);

//   navigation::generic::moveTowardsPosition(person.position, angle);
//   ROS_INFO("aMoveTowardsHuman - Moving towards Human ");

//   // while a goal is not cancelled execute the code
//   do {
//     // call the perception to retrieve the person position
//     // geometry_msgs::Pose tracker_pose = vision::generic::getTrackerPersonPose();
//     // set a goal to that person position
//     // navigation::generic::moveTowardsPosition(tracker_pose);
//   } while (navigation::generic::isMoveBaseGoal());
//   RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
//   *run = 1;
// }

void aMoveTowardsLocation(string params, bool* run) {
  // Move towards a certain location, not an object position
  // Navigation - Move towards a specific place
  string location = params;

  if (params == "Source") {
    GPSRActionsModel gpsrActionsDb;
    location = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::source_id);
  } else if (params == "Destination") {
    GPSRActionsModel gpsrActionsDb;
    location = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::destination_id);
  } else if (params == "WhereIsThis") {
    const string PARAM_NAME_WHEREIS_FURNITURE = "param_whereisthis_furniture";
    std_msgs::String FurnitureData;
    bool sqliteRet = SQLiteUtils::getParameterValue<std_msgs::String>(PARAM_NAME_WHEREIS_FURNITURE, FurnitureData);
    location = FurnitureData.data;
  } else {
    location = robobreizh::convertCamelCaseToSpacedText(params);
  }

  ROS_INFO("[aMoveTowardsLocation] - moving towards %s", location.c_str());

  robobreizh::database::LocationModel nm;
  robobreizh::database::Location np = nm.getLocationFromName(location);
  if (np.name.empty()) {
    ROS_ERROR("[aMoveTowardsLocation] Location name not found in the database and returned an empty location");
    dialog::generic::robotSpeech("Requested location is not found in the database, please fix this", 1);
    RoboBreizhManagerUtils::setPNPConditionStatus("NavQueryFailed");
  } else {
    navigation::generic::moveTowardsPosition(np.pose.position, np.angle);
    RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
  }
  *run = 1;
}

void aMoveTowardsHuman(string params, bool* run) {
  std::string human_name;
  if (params.empty()) {
    /*
        robobreizh::database::VisionModel vm;
        robobreizh::Person personPerson = vm.selectLastPerson();
        geometry_msgs::Point personPoint = {person.pos_x,person.pos_y,person.pos_z};

        ros::NodeHandle nh;
        nh.subscribe("/amcl_pose",geometry_msgs::PoseWithCovarianceStamped);
        geometry_msgs::PoseWithCovarianceStamped robotPose = ros::topic::waitForMessage("/amcl_pose",nh);
        int targetAngle = dialog::generic::getAngleABC(personPoint, robotPose.position, robotPose);
        navigation::generic::moveTowardsPosition(targetPose,(float)targetAngle);
        ROS_INFO("aMoveTowardsHuman - moving towards human");
  */
  } else if (params == "Stickler") {
    ROS_INFO("aMoveTowardsHuman - Moving towards current stickler person");
    std_msgs::Int32 stickler_id;
    SQLiteUtils::getParameterValue<std_msgs::Int32>("stickler_tracker_person_name", stickler_id);
    robobreizh::database::PersonModel pm;
    auto person = pm.getPerson(stickler_id.data);
    geometry_msgs::PoseWithCovariance robot_pose = navigation::generic::getCurrentPosition();

    float angle =
        std::atan2(person.position.y - robot_pose.pose.position.y, person.position.x - robot_pose.pose.position.x);

    navigation::generic::moveTowardsPosition(person.position, angle);

  } else if (params == "GPSR") {
    database::PersonModel pm;
    database::Person person = pm.getLastPerson();  // pm.getPersonByName(human_name);
    geometry_msgs::PoseWithCovariance robot_pose = navigation::generic::getCurrentPosition();

    float angle =
        std::atan2(person.position.y - robot_pose.pose.position.y, person.position.x - robot_pose.pose.position.x);

    navigation::generic::moveTowardsPosition(person.position, angle);
    ROS_INFO("aMoveTowardsHuman - Moving towards Human ");
  } else if (params == "lastPersonDb") {
    database::PersonModel pm;
    database::Person person = pm.getLastPerson();  // pm.getPersonByName(human_name);
    geometry_msgs::PoseWithCovariance robot_pose = navigation::generic::getCurrentPosition();

    float angle =
        std::atan2(person.position.y - robot_pose.pose.position.y, person.position.x - robot_pose.pose.position.x);

    navigation::generic::moveTowardsPosition(person.position, angle);
    ROS_INFO("aMoveTowardsHuman - Moving towards Human ");
  }
  RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
}

void aNavCheck(std::string params, bool* run) {
  // Check if the a destination is provided for navigation
  ROS_INFO("aNavCheck - Checking if a destination is provided for navigation");
  GPSRActionsModel gpsrActionsDb;
  string destination = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::destination_id);
  if (destination.empty()) {
    ROS_ERROR("aNavCheck - No destination provided for navigation");
    RoboBreizhManagerUtils::setPNPConditionStatus("NavFalse");
  } else {
    ROS_INFO("aNavCheck - Destination provided for navigation");
    RoboBreizhManagerUtils::setPNPConditionStatus("NavTrue");
  }
  *run = 1;
}

void aMoveTowardsGPSRTarget(string params, bool* run) {
  // Move towards a specific object, not a room location
  GPSRActionsModel gpsrActionsDb;
  string target_object = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item_id);

  ROS_INFO("aMoveTowardsGPSRTarget - Moving towards object %s", target_object.c_str());

  // Move towards target position
  // aMoveTowardsLocation(target, run);
}

void aRotate(string params, bool* run) {
  // Parse action parameters from "commands" parameter (not implemented yet)
  string str2;
  str2 = "minus";
  float angle = 0.0;

  if (params.find(str2) != string::npos) {
    params.erase(0, 5);
    angle = std::stod(params);
    angle = -angle;
  } else {
    angle = std::stod(params);
  }

  ROS_INFO("aRotate - turning %f", angle);

  navigation::generic::rotateOnPoint(angle);
  *run = 1;
}

void aTurnTowards(string params, bool* run) {
  // Parse action parameters from "commands" parameter (not implemented yet)
  string location = params;
  ROS_INFO("aTurnTowards - turning towards %s", location.c_str());

  // Move towards target
  *run = 1;
}

void aMoveBehindHuman(string params, bool* run) {
  RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
  *run = 1;
}

}  // namespace plan
}  // namespace navigation
}  // namespace robobreizh
