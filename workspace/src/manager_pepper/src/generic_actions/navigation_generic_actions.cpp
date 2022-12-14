#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <navigation_pep/NavigationDestination.h>
//#include <navigation_pep/AngleSrv.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include <navigation_pep/InitPose.h>

#include <boost/thread/thread.hpp>
#include "database_model/object_model.hpp"
#include "generic_actions/navigation_generic_actions.hpp"

using namespace std;

namespace robobreizh
{
namespace navigation
{
namespace generic
{
bool moveTowardsObject(string objectName)
{
  robobreizh::database::ObjectModel om;
  auto objPos = om.getPositionByLabel(objectName);
  geometry_msgs::Pose pose;
  pose.position = objPos.position;
  pose.orientation.w = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  moveTowardsPosition(pose, 0.0);
  return true;
}

bool convertThetaToQuat(float theta)
{
  /*tf2::Quaternion myQuaternion;

  myQuaternion.setRPY(0.0, 0.0, theta);

  myQuaternion.normalize();

  return myQuaternion;*/
  return true;
}

bool moveTowardsPosition(geometry_msgs::Pose p, float angle)
{
  return true;
}

bool rotateOnPoint(float angle)
{
  return true;
}

bool setInitPose(geometry_msgs::PoseWithCovarianceStamped p)
{
  return true;
}

}  // namespace generic
}  // namespace navigation
}  // namespace robobreizh
