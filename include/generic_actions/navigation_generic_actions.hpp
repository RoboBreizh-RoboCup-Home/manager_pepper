#ifndef _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace robobreizh {
namespace navigation {
namespace generic {
bool moveTowardsObject(std::string objectName /** Or object position if you prefer**/);
bool moveTowardsPosition(geometry_msgs::Pose p, float angle);
bool rotateOnPoint(float angle);
bool setInitPose(geometry_msgs::PoseWithCovarianceStamped p);

}  // namespace generic
}  // namespace navigation
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_
