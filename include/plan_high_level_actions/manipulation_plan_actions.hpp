#ifndef _PNP_ROBOBREIZH_MANIPULATION_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_MANIPULATION_PLAN_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh {
namespace manipulation {
namespace plan {
void aGrabHandle(std::string params, bool* run);
void aDropObject(std::string params, bool* run);
void aGraspObject(std::string params, bool* run);
void aPose(std::string params, bool* run);
void aCallMovementServer(std::string params, bool* run);
void aPutObject(std::string params, bool* run);
void aPourObject(std::string params, bool* run);
void aPullObject(std::string params, bool* run);
void aBendArms(std::string params, bool* run);
void aIsObjectCloseEnoughToGrasp(std::string params, bool* run);
void aStopMovement(std::string params, bool* run);
void aMoveArm(std::string params, bool* run);
void aSetHand(std::string params, bool* run);
}  // namespace plan
}  // namespace manipulation
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_MANIPULATION_PLAN_ACTIONS_
