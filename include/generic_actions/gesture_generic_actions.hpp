#ifndef _PNP_ROBOBREIZH_GESTURE_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_GESTURE_GENERIC_ACTIONS_
#include <geometry_msgs/PointStamped.h>

namespace robobreizh {
namespace gesture {
namespace generic {
bool lookUp();
bool lookDown();
bool lookAround();
bool pointInFront();
bool pointObjectPosition(geometry_msgs::PointStamped);
}  // namespace generic
}  // namespace gesture
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_GESTURE_GENERIC_ACTIONS_