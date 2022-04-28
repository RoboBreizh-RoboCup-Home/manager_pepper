#ifndef _PNP_ROBOBREIZH_NAVIGATION_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_NAVIGATION_PLAN_ACTIONS_

namespace robobreizh
{
    namespace navigation
    {
        namespace plan
        {
            void aMoveTowardsObject(std::string params, bool* run);
            void aFollowHuman(std::string params, bool* run);
        } // namespace plan
    } // namespace navigation
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_NAVIGATION_PLAN_ACTIONS_