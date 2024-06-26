#ifndef _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include "robobreizh_msgs/Object.h"
#include <robobreizh_msgs/Person.h>
#include <unordered_map>
#include <tf/tf.h>

#include "plan_high_level_actions/navigation_plan_actions.hpp"
#include "database_model/object_model.hpp"
#include "generic_actions/navigation_generic_actions.hpp"

namespace robobreizh {
namespace vision {
namespace generic {

enum ObjectServiceType { ALL, SHOES_DRINK_INFORMATION, BAG_INFORMATION, SEAT_INFORMATION };
enum Direction { LEFT, RIGHT, NONE };

bool findStoreSpecificObjectType(ObjectServiceType type);
// bool waitForHuman(double distanceMax);
bool waitForHuman();
std::vector<robobreizh::database::Person> findPersonPosition(float distance_max);
bool findObject(std::string objectName, database::Object* last_object_position);
Direction findDirectionPointedAt();
bool isDoorOpened();
bool findHumanAndStoreFeatures(robobreizh::database::Person* person);
int findHumanAndStoreFeaturesWithDistanceFilter(double distanceMax);
bool FindEmptySeat();
std::vector<std::string> findObjectsCategories();
std::unordered_map<std::string, int> countPose(std::unordered_map<std::string, std::string> WhatVariations);
std::string findAndLocateLastObjectPose();
bool WaitForHumanWavingHand();
#ifdef LEGACY
geometry_msgs::Pose getTrackerPersonPose();
bool findAndLocateCabDriver();
#endif
std::string matchPose(std::unordered_map<std::string, std::string> PersonVariations);
bool findHostAndStoreFeaturesWithDistanceFilter(double distanceMax);
bool breakTheRules(double distanceMax);
bool findHumanWithDrink(float distance_max);
bool findHumanWithShoes(float distance_max);
std::string FindObjectStoringGroceries();
}  // namespace generic
}  // namespace vision
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_VISION_GENERIC_ACTIONS_
