#ifndef _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include "perception_pepper/Object.h"
#include <tf/tf.h>

#include "plan_high_level_actions/navigation_plan_actions.hpp"
#include "database_model/object_model.hpp"
#include "generic_actions/navigation_generic_actions.hpp"

namespace robobreizh {
namespace vision {
namespace generic {

enum ObjectServiceType { ALL, SHOES_DRINK_INFORMATION, BAG_INFORMATION, SEAT_INFORMATION };

geometry_msgs::Pose getTrackerPersonPose();
bool findStoreSpecificObjectType(ObjectServiceType type);
// bool waitForHuman(double distanceMax);
bool waitForHuman();
bool findObject(std::string objectName, database::Object* last_object_position);
bool isDoorOpened();  // TODO: What if door not found ?
bool findHumanAndStoreFeatures(robobreizh::database::Person* person);
int findHumanAndStoreFeaturesWithDistanceFilter(double distanceMax);
bool FindEmptySeat();
std::string findAndLocateLastObjectPose();
bool WaitForHumanWavingHand();
#ifdef LEGACY
bool findAndLocateCabDriver();
#endif
bool findHostAndStoreFeaturesWithDistanceFilter(double distanceMax);
int breakTheRules(double distanceMax);
}  // namespace generic
}  // namespace vision
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
