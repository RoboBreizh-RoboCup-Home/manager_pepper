#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "GenericActions/NavigationGenericActions.hpp"
#include "ManagerUtils.hpp"
#include "DatabaseModel/NavigationModel.hpp"
#include "DatabaseModel/VisionModel.hpp"
#include "DatabaseModel/GPSRActionsModel.hpp"
#include "SQLiteUtils.hpp"

#include "geometry_msgs/Twist.h"

using namespace std;

using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

namespace robobreizh
{
namespace navigation
{
namespace plan
{
void aMoveTowardsObject(std::string params, bool* run)
{
  // Get Parameter(s)
  string object = params;
  ROS_INFO("aMoveTowardsObject - Currently moving torwards %s", object.c_str());

  // Navigation - Move towards a certain position
  navigation::generic::moveTowardsObject(object);

  RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);

  *run = 1;
}

void aFollowHuman(std::string params, bool* run)
{
  // Navigation - Follow human
  ROS_INFO("aFollowHuman - Following human");
  RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
  *run = 1;
}

void aMoveTowardsLocation(string params, bool* run)
{
  // Move towards a certain location, not an object position
  // Navigation - Move towards a specific place
  string location = params;

  if (params == "GPSR")
  {
    GPSRActionsModel gpsrActionsDb;
    location = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::destination);
  }
  else if (params == "WhereIsThis")
  {
    const string PARAM_NAME_WHEREIS_FURNITURE = "param_whereisthis_furniture";
    std_msgs::String FurnitureData;
    bool sqliteRet = SQLiteUtils::getParameterValue<std_msgs::String>(PARAM_NAME_WHEREIS_FURNITURE, FurnitureData);
    location = FurnitureData.data;
  }
  else
  {
    location = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(params);
  }

  ROS_INFO("aMoveTowardsLocation - moving towards %s", location.c_str());

  robobreizh::NavigationPlace np;
  robobreizh::database::NavigationModel nm;
  np = nm.getLocationFromName(location);

  navigation::generic::moveTowardsPosition(np.pose, np.angle);
  RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

/*
void aMoveStraight(){
       ros::NodeHandle nh; // handles node
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  geometry_msgs::Twist  msg;
  msg.linear.x = 1.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  ros::Time beginTime = ros::Time::now();
  ros::Duration MessageTime = ros::Duration(2);
  ros::Time endTime MessageTime = beginTime;

  while(ros::Time::now()< endTime){
    pub.publish(msg);
    ros::Duration(0.3).sleep();
  }
}
*/
void aMoveTowardsHuman(string params, bool* run)
{
  string humanName;
  if (params.empty())
  {
    /*
        robobreizh::database::VisionModel vm;
        robobreizh::Person personPerson = vm.selectLastPerson();
        geometry_msgs::Point personPoint = {person.pos_x,person.pos_y,person.pos_z};

        ros::NodeHandle nh;
        nh.subscribe("/amcl_pose",geometry_msgs::PoseWithCovarianceStamped);
        geometry_msgs::PoseWithCovarianceStamped robotPose = ros::topic::waitForMessage("/amcl_pose",nh);
        int targetAngle = dialog::generic::getAngleABC(personPoint, robotPose.position, robotPose);
        navigation::generic::moveTowardsPosition(targetPose,t(float)argetAngle);
        ROS_INFO("aMoveTowardsHuman - moving towards human");
  */
  }
  else if (params == "human")
  {
  }

  else
  {
    if (params == "GPSR")
    {
      GPSRActionsModel gpsrActionsDb;
      humanName = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
    }
    else
      humanName = params;
    ROS_INFO("aMoveTowardsHuman - Moving towards specific Human called %s", humanName.c_str());
  }
  RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
}

void aMoveTowardsGPSRTarget(string params, bool* run)
{
  // Move towards a specific object, not a room location
  GPSRActionsModel gpsrActionsDb;
  string target_object = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item);

  ROS_INFO("aMoveTowardsGPSRTarget - Moving towards object %s", target_object.c_str());

  // Move towards target position
  // aMoveTowardsLocation(target, run);
}

void aRotate(string params, bool* run)
{
  // Parse action parameters from "commands" parameter (not implemented yet)
  std::cout << params << std::endl;
  string str2;
  str2 = "minus";
  float angle = 0.0;

  if (params.find(str2) != string::npos)
  {
    params.erase(0, 5);
    angle = std::stod(params);
    angle = -angle;
  }
  else
  {
    angle = std::stod(params);
  }

  ROS_INFO("aRotate - turning %f", angle);

  navigation::generic::rotateOnPoint(angle);
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

void aTurnTowards(string params, bool* run)
{
  // Parse action parameters from "commands" parameter (not implemented yet)
  string location = params;
  ROS_INFO("aTurnTowards - turning towards %s", location.c_str());

  // Move towards target
  *run = 1;
}

void aMoveBehindHuman(string params, bool* run)
{
  RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
  *run = 1;
}

}  // namespace plan
}  // namespace navigation
}  // namespace robobreizh
