#ifndef _PNP_ROBOBREIZH_INITIALISATION_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_INITIALISATION_PLAN_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh
{
    namespace initialisation
    {
        namespace plan
        {
            void aInitCarryMyLuggage(std::string params, bool* run);
            void aInitGPSR(std::string params, bool* run);
            void aInitReceptionist(std::string params, bool* run);
            void aInitFindMyMate(std::string params, bool* run);
        } // namespace plan
    } // namespace initialisation
} // namespace robobreizh
#endif // _PNP_ROBOBREIZH_INITIALISATION_PLAN_ACTIONS_