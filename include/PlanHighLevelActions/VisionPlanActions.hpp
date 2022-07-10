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
            void aFindHuman(std::string params, bool* run);
            /* void aFindHumanFilter(std::string params, bool* run); */
            void aWaitForDoorOpening(std::string params, bool* run);
            void aFindEmptySeat(std::string params, bool* run);
            void aFindHumanAndStoreFeatures(std::string params, bool* run);
            void aFindHumanAndStoreFeaturesWithDistanceFilter(std::string params, bool* run);
            void aWaitForHumanWaivingHand(std::string params, bool* run);
            void aDialogAskHumanPlaceLastObjectOnTablet(std::string params, bool* run);
            void aLocatePositionToPlaceObject(std::string params, bool* run);
            void aFindCabDriver(std::string params, bool* run)
        } // namespace plan
    } // namespace vision
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_VISION_PLAN_ACTIONS_
