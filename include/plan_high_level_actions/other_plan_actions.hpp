#ifndef _PNP_ROBOBREIZH_OTHER_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_OTHER_PLAN_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh {
namespace other {
namespace plan {
void aGPSRProcessOrders(std::string params, bool* run);
void aIsHumanKnown(std::string params, bool* run);
void aCheckForMoreGuests(std::string params, bool* run);
void aChangePlan(std::string params, bool* run);
void aCheckForMoreObjectTofind(std::string params, bool* run);
void aWait(std::string params, bool* run);
void aChooseTake(std::string params, bool* run);
void aCheckObjectAndHuman(std::string params, bool* run);
}  // namespace plan
}  // namespace other
}  // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_OTHER_PLAN_ACTIONS_