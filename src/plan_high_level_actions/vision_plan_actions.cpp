#include <std_msgs/String.h>
#include <string>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <iostream>
#include <unordered_map>

#include <robobreizh_msgs/pointing_hand_detection.h>
#include <robobreizh_msgs/Person.h>
#include <robobreizh_msgs/Person.h>
#include "plan_high_level_actions/vision_plan_actions.hpp"
#include "generic_actions/vision_generic_actions.hpp"
#include "generic_actions/dialog_generic_actions.hpp"
#include "generic_actions/navigation_generic_actions.hpp"
#include "generic_actions/other_generic_actions.hpp"
#include "database_model/person_model.hpp"
#include "database_model/object_model.hpp"
#include "database_model/location_model.hpp"
#include "database_model/database_utils.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"
#include "vision_utils.hpp"
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

void aCheckNumsOfDetectionTime(string params, bool* run) {
  if (params == "stickler") {
    std_msgs::Int32 rotation_index;
    SQLiteUtils::getParameterValue<std_msgs::Int32>("rotation_index", rotation_index);
    if (rotation_index.data > 3) {
      g_stack_room.pop();
      std_msgs::Int32 room_index;
      room_index.data = g_stack_room.top();
      SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("room_index", room_index);
      // go to index room
      std::string room_name;
      switch (room_index.data) {
        case 0:
          ROS_ERROR("Room index should not be zero");
          break;
          // living room
        case 1:
          room_name = "stickler living room";
          break;
          // bedroom
        case 2:
          room_name = "stickler bedroom";
          break;
          // kitchen
        case 3:
          room_name = "stickler kitchen";
          break;
          // office
        case 4:
          room_name = "stickler office";
          break;

        default:
          ROS_ERROR("Room index is not valid");
          break;
      }

      robobreizh::database::LocationModel nm;
      robobreizh::database::Location np = nm.getLocationFromName(room_name);
      if (np.name.empty()) {
        ROS_ERROR("[aMoveTowardsLocation] Location name not found in the database and returned an empty location");
        np = nm.getLocationFromName("stickler living room");
        navigation::generic::moveTowardsPosition(np.pose.position, np.angle);
      } else {
        dialog::generic::robotSpeech("Moving to " + room_name, 1);
        navigation::generic::moveTowardsPosition(np.pose.position, np.angle);
      }
      rotation_index.data = 0;
      SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("rotation_index", rotation_index);
      RoboBreizhManagerUtils::setPNPConditionStatus("StopRotate");
    } else {
      RoboBreizhManagerUtils::setPNPConditionStatus("ContinueRotate");
    }
  } else {
    std_msgs::Int32 detection_number;
    std_msgs::Int32 counter_limit;

    SQLiteUtils::getParameterValue("detection_counter_limit", counter_limit);
    SQLiteUtils::getParameterValue("detection_number_record", detection_number);
    detection_number.data++;
    SQLiteUtils::modifyParameterParameter("detection_number_record", detection_number);

    std::cout << "detection_number = " << detection_number.data << " detection_counter_limit = " << counter_limit.data
              << std::endl;
    if (detection_number.data <= counter_limit.data) {
      ROS_INFO("Detection times: %d < Detection_limit: %d ", detection_number.data, counter_limit.data);
      RoboBreizhManagerUtils::setPNPConditionStatus("ContinueRotate");
    } else {
      ROS_WARN("No more Rotation for detection");
      detection_number.data = 0;
      SQLiteUtils::modifyParameterParameter("detection_number_record", detection_number);
      RoboBreizhManagerUtils::setPNPConditionStatus("StopRotate");
    }
  }
  *run = 1;
}

void aFindObject(string params, bool* run) {
  // Implement notFoundTimeout
  // Get parameters
  std::string objectToFind = params;
  std_msgs::Int32 detection_number;
  database::Object last_object;
  if (params == "GPSR") {
    GPSRActionsModel gpsrActionsDb;
    objectToFind = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item_id);

    // first detect in front of you
    if (generic::findObject(objectToFind, &last_object)) {
      RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
      // add the object to the database
      database::ObjectModel om;
      om.insertObject(last_object);
      // Object found: reset detection_number back to 0
      detection_number.data = 0;
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

void aMatchPose(std::string params, bool* run) {
  // Match the situation with the intent provided by the operator
  // get person related variations from database
  ros::NodeHandle nh;
  GPSRActionsModel gpsrActionsDb;
  std::unordered_map<std::string, std::string> variations;
  variations = gpsrActionsDb.getSpecificItemVariationsFromCurrentAction(GPSRActionItemName::what_id);

  if (params == "count") {
    std::unordered_map<std::string, int> countMap;
    countMap = robobreizh::vision::generic::countPose(variations);

    std::string pose = variations["descr_verb"];
    std::string direction = variations["descr_adj"];

    if (pose == "waving") {
      int countWave = countMap["waving"];
      dialog::generic::robotSpeech("There are " + std::to_string(countWave) + "people waving their hands", 0);
    }
    if (pose == "raising" and direction == "left") {
      int countRaisingLeft = countMap["raising_left_count"];
      dialog::generic::robotSpeech("There are " + std::to_string(countRaisingLeft) + "people waving their hands", 0);
    }

    if (pose == "raising" and direction == "right") {
      int countRaisingRight = countMap["raising_left_count"];
      dialog::generic::robotSpeech("There are " + std::to_string(countRaisingRight) + "people waving their hands", 0);
    }

    if (pose == "pointing" and direction == "left") {
      int countRaisingLeft = countMap["pointing_left_count"];
      dialog::generic::robotSpeech("There are " + std::to_string(countRaisingLeft) + "people waving their hands", 0);
    }
    if (pose == "pointing" and direction == "right") {
      int countRaisingRight = countMap["pointing_right_count"];
      dialog::generic::robotSpeech("There are " + std::to_string(countRaisingRight) + "people waving their hands", 0);
    }
    *run = 1;
  } else {
    std::string match = "";
    match = robobreizh::vision::generic::matchPose(variations);
    if (match != "") {
      RoboBreizhManagerUtils::setPNPConditionStatus("PoseMatched");
      // update person
      robobreizh::database::PersonModel pm;
      robobreizh::database::Person person;
      person.posture = match;
      person.is_drink = false;
      person.is_shoes = false;
      pm.insertPerson(person);
    } else {
      RoboBreizhManagerUtils::setPNPConditionStatus("PoseNotMatched");
    }
    *run = 1;
  }
}

void aFindHumanWithTimeout(string params, bool* run) {
  clock_t now = clock();
  bool getHuman = false;
  int timeout = std::stoi(params);
  string pnpStatus;

  do {
    getHuman = vision::generic::waitForHuman();
  } while ((!getHuman) && (clock() - now < timeout));

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

    double distanceMax = 3;
    if (params != "") {
      distanceMax = std::stod(params);
    }
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

void aFindObjectStoringGroceries(std::string params, bool* run) {
  std::string LastObjectOnTheTable;

  LastObjectOnTheTable = vision::generic::FindObjectStoringGroceries();

  if (LastObjectOnTheTable.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
    dialog::generic::robotSpeech("There is no object found on the table", 1);
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
    dialog::generic::robotSpeech("Can you carry the " + LastObjectOnTheTable + "for me please", 1);
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
  // std::String shelf_name = params;
  std::vector<std::string> categories;
  categories = vision::generic::findObjectsCategories();

  /*check if categories is empty*/
  if (categories.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("NoObjectFound");
    dialog::generic::robotSpeech("There is no object found on the shelf " + params, 1);
    *run = 1;
    return;
  }

  std_msgs::String relative_position_one;
  relative_position_one.data = params + " left";
  SQLiteUtils::storeNewParameter<std_msgs::String>(categories[0], relative_position_one);
  std_msgs::String relative_position_two;
  relative_position_two.data = params + " right";
  SQLiteUtils::storeNewParameter<std_msgs::String>(categories[1], relative_position_two);
  RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");

  // if (position != "") {
  //   dialog::generic::robotSpeech("Could you please place the object in the " + position, 1);
  // } else {
  //   dialog::generic::robotSpeech("Could you please place the object in the shelf 1.", 1);
  // }
  *run = 1;
}

void aFindObjectPosition(std::string params, bool* run) {
  // SQLiteUtils::getParameterValue<std_msgs::String>(categories[1], relative_position_two);

  // std_msgs::String objectToFind = params;
  database::Object last_object;
  if (vision::generic::findObject(params, &last_object)) {
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
    database::ObjectModel om;
    om.insertObject(last_object);
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
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
    navigation::generic::rotateOnPoint(-45.0);
  }
  // pointing_direction on robot's right
  else if (pointing_direction == vision::generic::LEFT) {
    navigation::generic::rotateOnPoint(45.0);
  } else {
    ROS_ERROR("Invalid Direction");
  }
  do {
    // Find object pointed by Human
    isTrue = vision::generic::findStoreSpecificObjectType(vision::generic::BAG_INFORMATION);
  } while ((!isTrue) && (clock() - now < 10));
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
  int timeout = std::stoi(params);
  string pnpStatus;

  do {
    // TODO Fill here
    shoesFound = vision::generic::findHumanWithShoes(3.0);
    // END TODO Fill here
  } while ((!shoesFound) && (clock() - now < timeout));

  if (shoesFound)
    pnpStatus = "ShoesFound";
  else
    pnpStatus = "NoShoesFound";

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

void aFindPersonWithoutDrink(std::string params, bool* run) {
  clock_t now = clock();
  bool drinkFound = false;
  float timeout = std::stoi(params);
  string pnpStatus;

  do {
    drinkFound = robobreizh::vision::generic::findHumanWithDrink(3.0);
  } while ((!drinkFound) && (clock() - now < timeout));

  if (drinkFound)
    pnpStatus = "DrinkFound";
  else
    pnpStatus = "NoDrinkFound";

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

void aFindPersonLittering(string params, bool* run) {
  clock_t now = clock();
  bool littering = false;
  int timeout = std::stoi(params);
  std::string pnpStatus;

  do {
    // TODO Fill here
    littering = true;
    // END TODO Fill here
  } while ((!littering) && (clock() - now < timeout));

  if (littering)
    pnpStatus = "LitteringFound";
  else
    pnpStatus = "NoLitteringFound";

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

void aFindStickler(string params, bool* run) {
  const double MAX_RANGE = 4;

  std::string pnpStatus = "None";

  auto how_much_time_since_start = ros::Time::now() - g_start;
  ROS_INFO("how_much_time_since_start = %f", how_much_time_since_start.toSec());

  // std_msgs::Int32 fr_attempt;
  // SQLiteUtils::getParameterValue<std_msgs::Int32>("forbidden_room_attempt", fr_attempt);
  // if (fr_attempt.data == 1) {
  //   if (how_much_time_since_start > ros::Duration(60 * 5)) {
  //     robobreizh::database::LocationModel nm;
  //     robobreizh::database::Location np = nm.getLocationFromName("stickler bedroom");
  //     if (np.name.empty()) {
  //       ROS_ERROR("[aMoveTowardsLocation] Location name not found in the database and returned an empty location");
  //       dialog::generic::robotSpeech("Requested location is not found in the database, please fix this", 1);
  //       RoboBreizhManagerUtils::setPNPConditionStatus("NavQueryFailed");
  //     } else {
  //       dialog::generic::robotSpeech("Moving to bedroom", 1);
  //       navigation::generic::moveTowardsPosition(np.pose.position, np.angle);
  //       RoboBreizhManagerUtils::setPNPConditionStatus("NavOK");
  //     }
  //   }
  // }

  if (!vision::generic::breakTheRules(MAX_RANGE)) {
    ROS_ERROR("Error: breakTheRules service failed to call");
    throw;
  }

  int result;
  int person_id;

  std_msgs::Int32 stickler_tracked_person;
  if (!robobreizh::other::generic::findWhoBreakTheRules(&person_id, &result)) {
    ROS_INFO_STREAM("No person breaking rule found");
    result = 0;
    stickler_tracked_person.data = -1;
  } else {
    ROS_INFO_STREAM("Person breaking rule found");
    stickler_tracked_person.data = person_id;
  }

  if (!SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("stickler_tracker_person_name",
                                                              stickler_tracked_person)) {
    ROS_ERROR_STREAM("Error while storing stickler_tracked_person");
  }

  switch (result) {
    case 0:
      pnpStatus = "None";
      break;
    case 1:
      pnpStatus = "Shoes";
      break;
    case 2:
      pnpStatus = "NoDrink";
      break;
    case 3: {
      std_msgs::Int32 fr_attempt;
      SQLiteUtils::getParameterValue<std_msgs::Int32>("forbidden_room_attempt", fr_attempt);
      if (fr_attempt.data == 0) {
        pnpStatus = "ForbiddenRoomFirstAttempt";
      } else {
        pnpStatus = "ForbiddenRoomSecondAttempt";
      }
      break;
    }
    case 4:
      pnpStatus = "Littering";
      break;
  }

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);

  std_msgs::Int32 rotation_index;
  SQLiteUtils::getParameterValue<std_msgs::Int32>("rotation_index", rotation_index);
  rotation_index.data = rotation_index.data + 1;
  SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("rotation_index", rotation_index);

  *run = 1;
}

void aFindPersonForbiddenRoom(string params, bool* run) {
  clock_t now = clock();
  bool drinkFound = false;
  float timeout = std::stoi(params);
  std::string pnpStatus = "None";

  do {
    std::vector<robobreizh::database::Person> persons = vision::generic::findPersonPosition(4.5);
    for (auto person : persons) {
      if (robobreizh::isInForbiddenRoom(person.position.x, person.position.y)) {
        pnpStatus = "ForbiddenRoom";
        break;
      }
    }
  } while ((pnpStatus != "ForbiddenRoom") && (clock() - now < timeout));

  *run = 1;
  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

}  // namespace plan
}  // namespace vision
}  // namespace robobreizh