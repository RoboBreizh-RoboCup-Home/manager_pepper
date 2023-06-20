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
bool pointObjectPosition(geometry_msgs::PointStamped, float distance);
bool look(std::vector<float> head_pitch_angle, std::vector<float> head_yaw_angle, std::vector<float> head_pitch_time,
          std::vector<float> head_yaw_time);
bool joint_angles(std::vector<std::string> joint_names, std::vector<float> joint_angles, float speed);
}  // namespace generic
}  // namespace gesture
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_GESTURE_GENERIC_ACTIONS_