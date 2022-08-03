#ifndef _PNP_ROBOBREIZH_MANIPULATION_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_MANIPULATION_PLAN_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh
{
namespace manipulation
{
namespace plan
{
void aGrabHandle(std::string params, bool* run);
void aDropObject(std::string params, bool* run);
void aLook(std::string params, bool* run);
void aPointAt(std::string params, bool* run);
void aBendArms(std::string params, bool* run);
}  // namespace plan
}  // namespace manipulation
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_MANIPULATION_PLAN_ACTIONS_