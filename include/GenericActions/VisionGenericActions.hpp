#ifndef _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include "DatabaseModel/VisionModel.hpp"
#include "perception_pepper/Object.h"
#include <tf/tf.h>
#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "DatabaseModel/VisionModel.hpp"
#include "GenericActions/NavigationGenericActions.hpp"

namespace robobreizh
{
    namespace vision
    {
        namespace generic
        {
			bool waitForHuman(double distanceMax);
            bool waitForHuman();
            bool findObject(std::string objectName); // bool is probably not the right output type, a pos seems more relevant
            bool isDoorOpened(); // TODO: What if door not found ?
			bool findHumanAndStoreFeatures(robobreizh::Person* person);
			int findHumanAndStoreFeaturesWithDistanceFilter(double distanceMax);
			bool FindEmptySeat();
			bool findStoreAllObjects();
			bool addObjectToDatabase(robobreizh::Object obj);
			bool addPersonToDatabase(robobreizh::Person person);
			bool isInRadius(float x1,float y1,float z1,float x2,float y2,float z2,float epsilon);
            std::string findObjectRange(std::string object, geometry_msgs::Point32 coord);
            std::string findAndLocateLastObjectPose();
            std::string findObjectCategory(std::string label);
            bool WaitForHumanWavingHand();
            bool findAndLocateBag();
			geometry_msgs::Point convertOdomToMap(float x,float y, float z);
            bool findAndLocateCabDriver();
            float convertOdomToBaseFootprint(float odomx, float odomy, float odomz);
            bool findHostAndStoreFeaturesWithDistanceFilter(double distanceMax);
            int breakTheRules(double distanceMax);
            int isInForbiddenRoom(float x, float y);
        } // namespace generic
    } // namespace vision
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
