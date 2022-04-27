#ifndef _PNP_ROBOBREIZH_MANIPULATION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_MANIPULATION_GENERIC_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh
{
    namespace manipulation
    {
        namespace generic
        {
            bool robotSpeech(std::string text);
            bool grabHandle(std::string object, std::string hand);
        } // namespace generic
    } // namespace manipulation
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_MANIPULATION_GENERIC_ACTIONS_