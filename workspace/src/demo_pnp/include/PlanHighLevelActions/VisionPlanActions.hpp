#ifndef _PNP_ROBOBREIZH_VISION_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_VISION_PLAN_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh
{
    namespace vision
    {
        namespace plan
        {
            void aWaitForOperator(std::string params, bool* run);
            void aFindObject(std::string params, bool* run);
        } // namespace plan
    } // namespace vision
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_VISION_PLAN_ACTIONS_