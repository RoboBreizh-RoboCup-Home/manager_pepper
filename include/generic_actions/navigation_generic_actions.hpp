#ifndef _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace robobreizh {
namespace navigation {
namespace generic {
extern std::vector<actionlib_msgs::GoalStatus> g_status;
void getStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
bool cancelGoal();
bool setInitPose(geometry_msgs::PoseWithCovarianceStamped p);
std::vector<actionlib_msgs::GoalStatus> getStatus();
bool isMoveBaseGoal();
bool moveTowardsObject(std::string objectName /** Or object position if you prefer**/);
bool moveTowardsPosition(geometry_msgs::Point p, float angle);
bool moveTowardsPosition(geometry_msgs::Pose p);
bool rotateOnPoint(float angle);
bool setInitPose(geometry_msgs::PoseWithCovarianceStamped p);
geometry_msgs::PoseWithCovariance getCurrentPosition();
}  // namespace generic
}  // namespace navigation
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_
