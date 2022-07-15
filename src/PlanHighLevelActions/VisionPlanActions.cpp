#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <iostream>

#include "PlanHighLevelActions/VisionPlanActions.hpp"
#include "GenericActions/VisionGenericActions.hpp"
#include "GenericActions/DialogGenericActions.hpp"
#include "DatabaseModel/VisionModel.hpp"
#include "ManagerUtils.hpp"
#include "SQLiteUtils.hpp"
#include "DatabaseModel/GPSRActionsModel.hpp"

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

void aFindObject(string params, bool* run)
{
    // Implement notFoundTimeout
    // Get parameters
    string objectToFind = params;
    if (params == "GPSR")
    {
        GPSRActionsModel gpsrActionsDb;
        objectToFind = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item);
        RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    }

    if (params == "All"){
        *run = vision::generic::findStoreAllObjects();
        RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);

    } else {
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

void aWaitForDoorOpening(string params, bool* run)
{
    bool doorOpened = false;
    do
    {
        doorOpened = vision::generic::isDoorOpened(); // TODO: Use Enum instead of bool (Open, closed, notfound)
    } while (!doorOpened); // TODO: Add timer for timeout
    RoboBreizhManagerUtils::setPNPConditionStatus("DoorFoundOpened");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = 1;
}

void aFindHumanAndStoreFeatures(string params, bool* run)
{
    bool getHuman = false;
    robobreizh::Person person;

    do
    {
        getHuman = vision::generic::findHumanAndStoreFeatures(&person); 
    } while (!getHuman); 

    RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");

    // format person in text
    RoboBreizhManagerUtils::pubVizBoxRobotText("gender : " + person.gender + ", age" + person.age + ", cloth color" + person.cloth_color + ", skin color : " + person.skin_color);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = 1;
}


void aFindHumanAndStoreFeaturesWithDistanceFilter(string params, bool* run)
{
    if (params == "host")
    {
        if (vision::generic::findHostAndStoreFeaturesWithDistanceFilter(4.0))
        {

            RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
        }
        else
        {
            // else rotate the robot
            RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotFound");
        }
    } else {
    int nbPerson;
    clock_t now = clock();
    
    double distanceMax = std::stod(params);
    do
    {
        nbPerson = vision::generic::findHumanAndStoreFeaturesWithDistanceFilter(distanceMax);
    } while (nbPerson <= 0);

    RoboBreizhManagerUtils::pubVizBoxRobotText("I found " + std::to_string(nbPerson) + "Persons in my field of view");
    // if human are detected look for objects
    if (nbPerson > 0){
        RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
        RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    }else {
        // else rotate the robot
        RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotFound");
    }
}
    *run = 1;
}


void aFindEmptySeat(std::string params, bool* run){
    bool isFree = false;
    isFree = vision::generic::FindEmptySeat(); 
    if (isFree){
        RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
        RoboBreizhManagerUtils::setPNPConditionStatus("EmptySeatFound");
    } else {
        RoboBreizhManagerUtils::setPNPConditionStatus("EmptySeatNotFound");
    }
    *run = 1;
}

void aWaitForHumanWaivingHand(string params, bool* run)
{
    // TODO: Wait for someone waiving hand
    RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
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
} // namespace plan
} // namespace vision
}// namespace robobreizh
