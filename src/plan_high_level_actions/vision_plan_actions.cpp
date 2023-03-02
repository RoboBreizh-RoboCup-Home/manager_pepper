#include <std_msgs/String.h>
#include <string>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <iostream>

#include "plan_high_level_actions/vision_plan_actions.hpp"
#include "generic_actions/vision_generic_actions.hpp"
#include "generic_actions/dialog_generic_actions.hpp"
#include "generic_actions/navigation_generic_actions.hpp"
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

namespace robobreizh
{
namespace vision
{
namespace plan
{
void aWaitForOperator(string params, bool* run)
{
  /*CV - Detect Human (no need to know their attributes such as gender, age, etc…)*/
  *run = vision::generic::waitForHuman();
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
}
bool isSubLocation(std::string location)
{
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

bool isAtSubLocation(std::string sub_location, std::string objectToFind)
{
  // move towards subplan location
  robobreizh::database::LocationModel lm;
  robobreizh::database::Location np = lm.getLocationFromName(sub_location);
  navigation::generic::moveTowardsPosition(np.pose, np.angle);
  // look for item
  if (vision::generic::findObject(objectToFind))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void aFindObject(string params, bool* run)
{
  // Implement notFoundTimeout
  // Get parameters
  std::string objectToFind = params;
  if (params == "GPSR")
  {
    GPSRActionsModel gpsrActionsDb;
    objectToFind = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item);

    robobreizh::database::ObjectModel om;

    // if db object exist
    if (om.getObjectByLabel(objectToFind).size() > 0)
    {
      // return  already exist
      RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
    }
    else
    {
      // list of all possible places where object can be
      std::vector<std::string> living_room_sub{ "couch table", "side table" };
      std::vector<std::string> kitchen_sub{ "pantry", "dinner table" };
      std::vector<std::string> bedroom_sub{ "small shelf" };
      std::vector<std::string> office_sub{ "desk" };

      // get current room
      std::string location = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::destination);
      // if is a sub location
      if (isSubLocation(location))
      {
        if (vision::generic::findObject(objectToFind))
        {
          RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
        }
      }
      else
      {
        if (location == "living room")
        {
          for (std::string sub_location : living_room_sub)
          {
            if (isAtSubLocation(sub_location, objectToFind))
            {
              RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
            }
            else
            {
              RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
            }
          }
        }
        else if (location == "kitchen")
        {
          for (std::string sub_location : kitchen_sub)
          {
            if (isAtSubLocation(sub_location, objectToFind))
            {
              RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
            }
            else
            {
              RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
            }
          }
        }
        else if (location == "bedroom")
        {
          for (std::string sub_location : bedroom_sub)
          {
            if (isAtSubLocation(sub_location, objectToFind))
            {
              RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
            }
            else
            {
              RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
            }
          }
        }
        else if (location == "office")
        {
          for (std::string sub_location : office_sub)
          {
            if (isAtSubLocation(sub_location, objectToFind))
            {
              RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
            }
            else
            {
              RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
              ROS_ERROR("ObjectNotFound");
            }
          }
        }
        else
        {
          ROS_ERROR("the destination is not a room name but went in the room name switch case");
        }
      }
    }
    *run = 1;
  }

  else if (params == "All")
  {
    *run = vision::generic::findStoreAllObjects();
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  }
  else
  {
    /* CV - Detect luggage */
    ROS_INFO("FindObject - Currently looking for %s", objectToFind.c_str());
    *run = vision::generic::findObject(objectToFind);
  }
}

void aFindHumanFilter(std::string params, bool* run)
{
  bool getHuman = false;
  double distanceMax = std::stod(params);
  /* do */
  /* { */
  /*     getHuman = vision::generic::waitForHuman(distanceMax); */
  /* } while (!getHuman); */

  RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

void aFindHuman(std::string params, bool* run)
{
  // ask to be in front
  dialog::generic::robotSpeech("Could you please look at me");
  if (params.empty())
  {
    // Find any Human
    bool getHuman = false;
    do
    {
      getHuman = vision::generic::waitForHuman();
    } while (!getHuman);
  }

  else if (params == "new")
  {
    // TODO Find human not already on the database
    ROS_INFO("aFindHuman - Find a new Human not already on the database");
  }

  else
  {
    // For example, find the Host however the guest is nearby
    ROS_INFO("aFindHuman - Find specific Human called %s", params.c_str());
  }

  RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

void aFindHumanWithTimeout(string params, bool* run)
{
  clock_t now = clock();
  bool getHuman = false;
  int timeout = stoi(params);
  string pnpStatus;

  do
  {
    getHuman = vision::generic::waitForHuman();
  } while ((!getHuman) || (clock() - now < timeout));

  if (getHuman)
    pnpStatus = "HumanFound";
  else
    pnpStatus = "NoHumanFound";

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
}

void aWaitForDoorOpening(string params, bool* run)
{
  bool doorOpened = false;
  do
  {
    doorOpened = vision::generic::isDoorOpened();  // TODO: Use Enum instead of bool (Open, closed, notfound)
  } while (!doorOpened);                           // TODO: Add timer for timeout
  RoboBreizhManagerUtils::setPNPConditionStatus("DoorFoundOpened");
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

void aFindHumanAndStoreFeatures(string params, bool* run)
{
  bool getHuman = false;
  robobreizh::database::Person person;

  do
  {
    getHuman = vision::generic::findHumanAndStoreFeatures(&person);
  } while (!getHuman);

  RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");

  // format person in text
  RoboBreizhManagerUtils::pubVizBoxRobotText("gender : " + person.gender + ", age" + person.age + ", cloth color" +
                                             person.cloth_color.label + ", skin color : " + person.skin_color.label);
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

void aFindHumanAndStoreFeaturesWithDistanceFilter(string params, bool* run)
{
  if (params == "host")
  {
    if (vision::generic::findHostAndStoreFeaturesWithDistanceFilter(4.0))
    {
      ROS_INFO("Host detection with distance success");
      RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
    }
    else
    {
      // else rotate the robot
      ROS_INFO("Hos detection with distance failed");
      RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotFound");
    }
  }
  else
  {
    int nbPerson;

    double distanceMax = std::stod(params);

    nbPerson = vision::generic::findHumanAndStoreFeaturesWithDistanceFilter(distanceMax);

    // if human are detected look for objects
    if (nbPerson > 0)
    {
      ROS_INFO("%s Persons found within %f m range and features were stored", std::to_string(nbPerson), distanceMax);
      RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
    }
    else
    {
      // else rotate the robot
      ROS_WARN("0 Person were found");
      RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotFound");
    }
  }
  *run = 1;
}

void aFindEmptySeat(std::string params, bool* run)
{
  bool isFree = false;
  isFree = vision::generic::FindEmptySeat();
  if (isFree)
  {
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::setPNPConditionStatus("EmptySeatFound");
  }
  else
  {
    RoboBreizhManagerUtils::setPNPConditionStatus("EmptySeatNotFound");
  }
  *run = 1;
}

void aWaitForHumanWavingHand(string params, bool* run)
{
  // Specific cases
  if (params == "eraseDbFirst")
  {
    robobreizh::database::PersonModel pm;
    pm.clearPerson();
  }

  // TODO: Wait for someone waiving hand
  bool isTrue;
  clock_t now = clock();

  do
  {
    isTrue = vision::generic::WaitForHumanWavingHand();
  } while ((!isTrue) || (clock() - now < 15));
  if (isTrue)
  {
    RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
  }
  else
  {
    RoboBreizhManagerUtils::setPNPConditionStatus("HNotFound");
  }
}

void aLocatePositionToPlaceObject(std::string params, bool* run)
{
  std::string position;
  position = vision::generic::findAndLocateLastObjectPose();

  if (position != "")
  {
    dialog::generic::robotSpeech("Could you please place the object in the " + position);
  }
  else
  {
    dialog::generic::robotSpeech("Could you please place the object in the shelf 1.");
  }
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

void aFindCabDriver(string params, bool* run)
{
  // Check if driver is already on database

  // If driver not already on database, find them (someone wearing fluorescent colours and/or standing under an open
  // umbrella)

  // Store driver name and position on database as CabDriver
  bool unefoissurdeux;
  unefoissurdeux = true;
  if (vision::generic::findAndLocateCabDriver())
  {
    *run = 1;
    return;
  }
  do
  {
    if (unefoissurdeux)
    {
      system("rosservice call /robobreizh/manipulation/look_right");
      unefoissurdeux = false;
    }
    else
    {
      system("rosservice call /robobreizh/manipulation/look_right");
      unefoissurdeux = true;
    }

  } while (vision::generic::findAndLocateCabDriver());
  RoboBreizhManagerUtils::setPNPConditionStatus("DriverFound");
  *run = 1;
}

void aFindObjectPointedByHuman(string params, bool* run)
{
  // If found, store pointed object by Human in the database using the name "bag"
  bool isTrue;
  clock_t now = clock();

  do
  {
    // Find object pointed by Human
    // isTrue = vision::generic::WaitForHumanWavingHand();
    isTrue = true;
  } while ((!isTrue) || (clock() - now < 15));
  if (isTrue)
  {
    // If found, store pointed object by Human in the database using the name "bag"

    // Update PNP condition status
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
  }
  else
  {
    RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
  }
}

void aFindPersonWithShoes(string params, bool* run)
{
  clock_t now = clock();
  bool shoesFound = false;
  int timeout = stoi(params);
  string pnpStatus;

  do
  {
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

void aFindPersonWithoutDrink(std::string params, bool* run)
{
  clock_t now = clock();
  bool noDrinkFound = false;
  float timeout = stoi(params);
  string pnpStatus;

  do
  {
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

void aFindPersonLittering(string params, bool* run)
{
  clock_t now = clock();
  bool littering = false;
  int timeout = stoi(params);
  string pnpStatus;

  do
  {
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

void aFindStickler(string params, bool* run)
{
  const double MAX_RANGE = 4;

  string pnpStatus = "None";

  int result = vision::generic::breakTheRules(MAX_RANGE);

  switch (result)
  {
    case 0:
      pnpStatus = "None";
      break;
    case 1:
      pnpStatus = "Shoes";
      break;
    case 2:
      pnpStatus = "NoDrink";
      break;
    case 3:
      pnpStatus = "ForbiddenRoom";
      break;
    case 4:
      pnpStatus = "Littering";
      break;
  }

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
  *run = 1;
}

void aFindPersonForbiddenRoom(string params, bool* run)
{
  *run = 1;
}
}  // namespace plan
}  // namespace vision
}  // namespace robobreizh
