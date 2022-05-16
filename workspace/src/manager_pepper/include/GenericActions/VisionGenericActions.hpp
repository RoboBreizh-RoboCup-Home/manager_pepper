#ifndef _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include <perception_pepper/Object.h>


namespace robobreizh
{
    namespace vision
    {
        namespace generic
        {
            bool getObjectIfAlreadyBeenFound(const std::string &objName, perception_pepper::Object &obj);
            bool waitForHuman();
            bool findObject(std::string objectName); // bool is probably not the right output type, a pos seems more relevant
            bool isDoorOpened(); // TODO: What if door not found ?
        } // namespace generic
    } // namespace vision
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_