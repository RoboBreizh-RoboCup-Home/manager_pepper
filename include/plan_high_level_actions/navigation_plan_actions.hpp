#ifndef _PNP_ROBOBREIZH_NAVIGATION_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_NAVIGATION_PLAN_ACTIONS_

namespace robobreizh {
namespace navigation {
namespace plan {
void aNavCheck(std::string params, bool* run);
void aMoveTowardsObject(std::string params, bool* run);
void aFollowHuman(std::string params, bool* run);
void aMoveTowardsLocation(std::string params, bool* run);
void aMoveTowardsHuman(std::string params, bool* run);
void aMoveTowardsGPSRTarget(std::string params, bool* run);
void aRotate(std::string params, bool* run);
void aTurnTowards(std::string params, bool* run);
void aMoveBehindHuman(std::string params, bool* run);
// void aMoveStraight(std::string params, bool* run);
}  // namespace plan
}  // namespace navigation
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_NAVIGATION_PLAN_ACTIONS_
