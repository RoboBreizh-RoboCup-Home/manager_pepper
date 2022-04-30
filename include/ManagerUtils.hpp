#ifndef _PNP_ROBOBREIZH_MANAGER_UTILS_
#define _PNP_ROBOBREIZH_MANAGER_UTILS_

#include <std_msgs/String.h>

namespace robobreizh
{
    class RoboBreizhManagerUtils
    {
    public:
        RoboBreizhManagerUtils() = default;
        ~RoboBreizhManagerUtils() = default;

        static std::string getPNPConditionStatus();
        static bool setPNPConditionStatus(std::string status);
    };
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_MANAGER_UTILS_