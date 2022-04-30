#ifndef _PNP_ROBOBREIZH_DIALOG_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_DIALOG_PLAN_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh
{
    namespace dialog
    {
        namespace plan
        {
            void aGreetHuman(std::string params, bool* run);
            void aAskHandOverObject(std::string params, bool* run);
            void aTellReadyToGo(std::string params, bool* run);
            void aTellGoodbye(std::string params, bool* run);
            void aAskHuman(std::string params, bool* run);
            void aListenOrders(std::string params, bool* run);        
        } // namespace generic
    } // namespace plan
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_DIALOG_PLAN_ACTIONS_