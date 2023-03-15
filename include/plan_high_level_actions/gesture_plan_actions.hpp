#ifndef _PNP_ROBOBREIZH_GESTURE_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_GESTURE_PLAN_ACTIONS_

#include <string>

namespace robobreizh {
namespace gesture {
namespace plan {
void aLookAt(std::string params, bool* run);
void aLook(std::string params, bool* run);
void aPointAt(std::string params, bool* run);
void aBendArms(std::string params, bool* run);
}  // namespace plan
}  // namespace gesture
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_GESTURE_PLAN_ACTIONS_
