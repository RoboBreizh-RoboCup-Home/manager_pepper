#ifndef _PNP_ROBOBREIZH_OTHER_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_OTHER_GENERIC_ACTIONS_

namespace robobreizh
{
    namespace other
    {
        namespace generic
        {
            bool waitForGoSignal();
            bool isValidObject(std::string objName);
            bool isValidPlace(std::string placeName);
        } // namespace generic
    } // namespace other
}// namespace robobreizh
#endif // _PNP_ROBOBREIZH_OTHER_GENERIC_ACTIONS_