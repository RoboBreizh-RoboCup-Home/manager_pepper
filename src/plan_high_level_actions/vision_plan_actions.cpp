#include <std_msgs/String.h>
#include <string>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <iostream>

#include <robobreizh_msgs/pointing_hand_detection.h>
#include "plan_high_level_actions/vision_plan_actions.hpp"
#include "generic_actions/vision_generic_actions.hpp"
#include "generic_actions/dialog_generic_actions.hpp"
#include "generic_actions/navigation_generic_actions.hpp"
#include "generic_actions/other_generic_actions.hpp"
#include "database_model/person_model.hpp"
#include "database_model/object_model.hpp"
#include "database_model/location_model.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"
#include "database_model/gpsr_actions_model.hpp"
#include <ctime>

using namespace std;

using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

namespace robobreizh {
namespace vision {
namespace plan {
void aWaitForOperator(string params, bool* run) {
  /*CV - Detect Human (no need to know their attributes such as gender, age, etc…)*/
  *run = vision::generic::waitForHuman();
  RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
}
bool isSubLocation(std::string location) {
  // std::vector<std::string> sub {"house plant", "coat rack", "sofa", "crouch table", "tv", "side table","book
  // shelf","kitchen shelf","pantry","dinner table","kitchen bin","fridge","washing machine","sink","small
  // shelf","cupboard","big shelf","bed","desk","show rack","bin","office shelf"};
  std::vector<std::string> rooms{ "living room", "office", "bedroom", "kitchen" };
  // if any of main location return false
  if (std::find(rooms.begin(), rooms.end(), location) != rooms.end())
    return false;
  else
    return true;
}

bool isAtSubLocation(std::string sub_location, std::string objectToFind) {
  // move towards subplan location
  robobreizh::database::LocationModel lm;
  robobreizh::database::Location np = lm.getLocationFromName(sub_location);
  navigation::generic::moveTowardsPosition(np.pose.position, np.angle);
  // look for item
  database::Object last_object;
  if (vision::generic::findObject(objectToFind, &last_object)) {
    return true;
  } else {
    return false;
  }
}

void aFindObject(string params, bool* run) {
  // Implement notFoundTimeout
  // Get parameters
  std::string objectToFind = params;
  database::Object last_object;
  if (params == "GPSR") {
    GPSRActionsModel gpsrActionsDb;
    objectToFind = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item);

    // first detect in front of you
    if (generic::findObject(objectToFind, &last_object)) {
      RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
      // add the object to the database
      database::ObjectModel om;
      om.insertObject(last_object);
    } else {
      RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
    }
    *run = 1;
  } else if (params == "All") {
    robobreizh::vision::generic::findStoreSpecificObjectType(robobreizh::vision::generic::ObjectServiceType::ALL);
    *run = 1;
  } else {
    ROS_INFO("FindObject - Currently looking for %s", objectToFind.c_str());
    if (vision::generic::findObject(objectToFind, &last_object)) {
      RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
      database::ObjectModel om;
      om.insertObject(last_object);
    } else {
      RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
    }
    *run = 1;
  }
}

void aFindHumanFilter(std::string params, bool* run) {
  bool getHuman = false;
  double distanceMax = std::stod(params);
  /* do */
  /* { */
  /*     getHuman = vision::generic::waitForHuman(distanceMax); */
  /* } while (!getHuman); */

  RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
  *run = 1;
}

void aFindHuman(std::string params, bool* run) {
  // ask to be in front
  dialog::generic::robotSpeech("Could you please look at me", 1);
  bool getHuman = vision::generic::waitForHuman();
  if (getHuman) {
    RoboBreizhManagerUtils::setPNPConditionStatus("HumanFound");
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotFound");
  }
  *run = 1;
}

void aFindHumanWithTimeout(string params, bool* run) {
  clock_t now = clock();
  bool getHuman = false;
  int timeout = stoi(params);
  string pnpStatus;

  do {
    getHuman = vision::generic::waitForHuman();
  } while ((!getHuman) || (clock() - now < timeout));

  if (getHuman)
    pnpStatus = "HumanFound";
  else
    pnpStatus = "NoHumanFound";

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

void aWaitForDoorOpening(string params, bool* run) {
  bool doorOpened = false;
  do {
    doorOpened = vision::generic::isDoorOpened();  // TODO: Use Enum instead of bool (Open, closed, notfound)
  } while (!doorOpened);                           // TODO: Add timer for timeout
  RoboBreizhManagerUtils::setPNPConditionStatus("DoorFoundOpened");
  *run = 1;
}

void aFindHumanAndStoreFeatures(string params, bool* run) {
  bool getHuman = false;
  robobreizh::database::Person person;

  do {
    getHuman = vision::generic::findHumanAndStoreFeatures(&person);
  } while (!getHuman);

  RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");

  // format person in text
  RoboBreizhManagerUtils::pubVizBoxRobotText("gender : " + person.gender + ", age" + person.age + ", cloth color" +
                                             person.cloth_color.label + ", skin color : " + person.skin_color.label);
  *run = 1;
}

void aFindHumanAndStoreFeaturesWithDistanceFilter(string params, bool* run) {
  if (params == "host") {
    if (vision::generic::findHostAndStoreFeaturesWithDistanceFilter(4.0)) {
      ROS_INFO("Host detection with distance success");
      RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
    }
    // else {
    //   // else rotate the robot
    //   ROS_INFO("Hos detection with distance failed");
    //   RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotFound");
    // }
  } else {
    int nbPerson;

    double distanceMax = std::stod(params);

    nbPerson = vision::generic::findHumanAndStoreFeaturesWithDistanceFilter(distanceMax);

    // if human are detected look for objects
    if (nbPerson > 0) {
      ROS_INFO("%s Persons found within %f m range and features were stored", std::to_string(nbPerson).c_str(),
               distanceMax);
      RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
    } else {
      // else rotate the robot
      ROS_WARN("0 Person were found");
      RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotFound");
    }
  }
  *run = 1;
}

void aFindEmptySeat(std::string params, bool* run) {
  bool isFree = false;
  isFree = vision::generic::FindEmptySeat();
  if (isFree) {
    RoboBreizhManagerUtils::setPNPConditionStatus("EmptySeatFound");
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("EmptySeatNotFound");
  }
  *run = 1;
}

void aWaitForHumanWavingHand(string params, bool* run) {
  // Specific cases
  if (params == "eraseDbFirst") {
    robobreizh::database::PersonModel pm;
    pm.clearPerson();
  }

  // TODO: Wait for someone waiving hand
  bool isTrue;
  clock_t now = clock();

  do {
    isTrue = vision::generic::WaitForHumanWavingHand();
  } while ((!isTrue) || (clock() - now < 15));
  if (isTrue) {
    navigation::generic::cancelGoal();
    RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("HNotFound");
  }
}

void aLocatePositionToPlaceObject(std::string params, bool* run) {
  std::string position;
  position = vision::generic::findAndLocateLastObjectPose();

  if (position != "") {
    dialog::generic::robotSpeech("Could you please place the object in the " + position, 1);
  } else {
    dialog::generic::robotSpeech("Could you please place the object in the shelf 1.", 1);
  }
  *run = 1;
}

#ifdef LEGACY
void aFindCabDriver(string params, bool* run) {
  // Check if driver is already on database

  // If driver not already on database, find them (someone wearing fluorescent colours and/or standing under an open
  // umbrella)

  // Store driver name and position on database as CabDriver
  bool unefoissurdeux;
  unefoissurdeux = true;
  if (vision::generic::findAndLocateCabDriver()) {
    *run = 1;
    return;
  }
  do {
    if (unefoissurdeux) {
      system("rosservice call /robobreizh/manipulation/look_right");
      unefoissurdeux = false;
    } else {
      system("rosservice call /robobreizh/manipulation/look_right");
      unefoissurdeux = true;
    }

  } while (vision::generic::findAndLocateCabDriver());
  RoboBreizhManagerUtils::setPNPConditionStatus("DriverFound");
  *run = 1;
}
#endif

void aFindObjectPointedByHuman(string params, bool* run) {
  // If found, store pointed object by Human in the database using the name "bag"
  bool isTrue;
  clock_t now = clock();
  // turn to right direction before finding the object
  bool pointing_direction = vision::generic::findDirectionPointedAt();
  // pointing_direction on robot's left
  if (pointing_direction == vision::generic::RIGHT) {
    navigation::generic::rotateOnPoint(-35.0);
  }
  // pointing_direction on robot's right
  else if (pointing_direction == vision::generic::LEFT) {
    navigation::generic::rotateOnPoint(35.0);
  } else {
    ROS_ERROR("Invalid Direction");
  }
  do {
    // Find object pointed by Human
    isTrue = vision::generic::findStoreSpecificObjectType(vision::generic::BAG_INFORMATION);
  } while ((!isTrue) || (clock() - now < 10));
  if (isTrue) {
    // If found, store pointed object by Human in the database using the name "bag"

    // Update PNP condition status
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
  }
}

void aFindPersonWithShoes(string params, bool* run) {
  clock_t now = clock();
  bool shoesFound = false;
  int timeout = stoi(params);
  string pnpStatus;

  do {
    // TODO Fill here
    shoesFound = true;
    // END TODO Fill here
  } while ((!shoesFound) || (clock() - now < timeout));

  if (shoesFound)
    pnpStatus = "ShoesFound";
  else
    pnpStatus = "NoShoesFound";

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

void aFindPersonWithoutDrink(std::string params, bool* run) {
  clock_t now = clock();
  bool noDrinkFound = false;
  float timeout = stoi(params);
  string pnpStatus;

  do {
    // TODO Fill here
    noDrinkFound = true;
    // END TODO Fill here
  } while ((!noDrinkFound) || (clock() - now < timeout));

  if (noDrinkFound)
    pnpStatus = "NoDrinkFound";
  else
    pnpStatus = "DrinkFound";

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

void aFindPersonLittering(string params, bool* run) {
  clock_t now = clock();
  bool littering = false;
  int timeout = stoi(params);
  string pnpStatus;

  do {
    // TODO Fill here
    littering = true;
    // END TODO Fill here
  } while ((!littering) || (clock() - now < timeout));

  if (littering)
    pnpStatus = "LitteringFound";
  else
    pnpStatus = "NoLitteringFound";

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

void aFindStickler(string params, bool* run) {
  const double MAX_RANGE = 4;

  std::string pnpStatus = "None";

  if (!vision::generic::breakTheRules(MAX_RANGE)) {
    ROS_ERROR("Error: breakTheRules service failed to call");
    throw;
  }

  int result;
  int person_id;

  if (!robobreizh::other::generic::findWhoBreakTheRules(&person_id, &result)) {
    ROS_INFO_STREAM("No person breaking rulefound");
    result = 0;
    std::string stickler_tracker_person_name = "stickler_tracker_person_name";
    std_msgs::Int32 stickler_tracked_person;
    stickler_tracked_person.data = -1;
    SQLiteUtils::storeNewParameter<std_msgs::Int32>(stickler_tracker_person_name, stickler_tracked_person);
  } else {
    ROS_INFO_STREAM("Person breaking rule found");
    std::string stickler_tracker_person_name = "stickler_tracker_person_name";
    std_msgs::Int32 stickler_tracked_person;
    stickler_tracked_person.data = person_id;
    SQLiteUtils::storeNewParameter<std_msgs::Int32>(stickler_tracker_person_name, stickler_tracked_person);
  }
  ROS_INFO_STREAM("result: " << result);

  // switch (result) {
  //   case 0:
  //     pnpStatus = "None";
  //     break;
  //   case 1:
  //     pnpStatus = "Shoes";
  //     break;
  //   case 2:
  //     pnpStatus = "NoDrink";
  //     break;
  //   case 3:
  //     pnpStatus = "ForbiddenRoom";
  //     break;
  //   case 4:
  //     pnpStatus = "Littering";
  //     break;
  // }

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
  *run = 1;
}

void aFindPersonForbiddenRoom(string params, bool* run) {
  *run = 1;
}
}  // namespace plan
}  // namespace vision
}  // namespace robobreizh
