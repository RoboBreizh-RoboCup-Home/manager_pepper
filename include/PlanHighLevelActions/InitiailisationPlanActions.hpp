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
            void aInitRestaurant(std::string params, bool* run);
            void sendPlanVizbox(std::string title, std::vector<std::string> storyline);
            void aInitFarewell(std::string params,bool*run);
            void aInitStoringGroceries(std::string params,bool*run);
        } // namespace plan
    } // namespace initialisation
} // namespace robobreizh
#endif // _PNP_ROBOBREIZH_INITIALISATION_PLAN_ACTIONS_
