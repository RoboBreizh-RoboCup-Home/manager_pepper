#ifndef _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh
{
    namespace navigation
    {
        namespace generic
        {
            bool moveTowardsObject(std::string objectName /** Or object position if you prefer**/);
        } // namespace generic
    } // namespace navigation
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_NAVIGATION_GENERIC_ACTIONS_