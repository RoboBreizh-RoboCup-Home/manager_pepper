#ifndef _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include "DatabaseModel/VisionModel.hpp"

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
			geometry_msgs::Point convertOdomToMap(float x,float y, float z);
        } // namespace generic
    } // namespace vision
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
