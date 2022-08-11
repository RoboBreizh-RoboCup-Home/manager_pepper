#include <ros/ros.h>
#include <std_msgs/String.h>

#include "plan_high_level_actions/gesture_plan_actions.hpp"

using namespace std;

namespace robobreizh
{
namespace gesture
{
namespace plan
{
void aLookAt(string params, bool* run)
{
  *run = 1;
}
}  // namespace plan
}  // namespace gesture
}  // namespace robobreizh
