#ifndef _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh
{
    namespace dialog
    {
        namespace generic
        {
            bool robotSpeech(std::string text);
            std::vector<std::string> ListenSpeech();
            std::vector<std::string> wavToIntent();
        } // namespace generic
    } // namespace dialog
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_
