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
#include "DatabaseModel/InitModel.hpp"
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

void aFindObject(string params, bool* run)
{
    // Implement notFoundTimeout
    // Get parameters
    string objectToFind = params;
    if (params == "GPSR")
    {
        GPSRActionsModel gpsrActionsDb;
        objectToFind = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item);
    }

    if (params == "All"){
        *run = vision::generic::findStoreAllObjects();

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
    int nbPerson;
    
    double distanceMax = std::stod(params);

    nbPerson = vision::generic::findHumanAndStoreFeaturesWithDistanceFilter(distanceMax); 

    RoboBreizhManagerUtils::pubVizBoxRobotText("I found " + std::to_string(nbPerson) + "Persons in my field of view");
    // if human are detected look for objects
    if (nbPerson > 0){
        RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
    }else {
        // else rotate the robot
        RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotFound");
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

void aWaitForHumanWavingHand(string params, bool* run)
{
    // Specific cases
    if (params == "eraseDbFirst")
    {
        robobreizh::database::InitModel im;
        im.deleteAllPerson();
    }

    // TODO: Wait for someone waiving hand
    bool isTrue;
    clock_t now = clock();

    do
    {
        isTrue = vision::generic::WaitForHumanWavingHand();
    } while ((!isTrue)|| (clock() - now < 15));
    if(isTrue){
        RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
    }else{
        RoboBreizhManagerUtils::setPNPConditionStatus("HNotFound");
    }
}

void aLocatePositionToPlaceObject(std::string params, bool* run)
{
    std::string position;
    position = vision::generic::findAndLocateLastObjectPose(); 
    
    if(position != ""){
        dialog::generic::robotSpeech("Could you please place the object in the "+position);
    }else{
        dialog::generic::robotSpeech("Could you please place the object in the shelf 1.");
    }
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = 1;
}

void aFindCabDriver(string params, bool* run)
{
    // Check if driver is already on database

    // If driver not already on database, find them (someone wearing fluorescent colours and/or standing under an open umbrella)

    // Store driver name and position on database as CabDriver
    bool unefoissurdeux;
    unefoissurdeux = true;
    if(vision::generic::findAndLocateCabDriver()){
        *run = 1;
        return;
    }
    do
    {
        if(unefoissurdeux){
            system("rosservice call /robobreizh/manipulation/look_right");
            unefoissurdeux = false;
 
        }else{
            system("rosservice call /robobreizh/manipulation/look_right");
            unefoissurdeux = true;
        }

    }while(vision::generic::findAndLocateCabDriver());
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
        //isTrue = vision::generic::WaitForHumanWavingHand();
        isTrue = true;
    } while ((!isTrue) || (clock() - now < 15));
    if(isTrue){
        // If found, store pointed object by Human in the database using the name "bag"

        // Update PNP condition status
        RoboBreizhManagerUtils::setPNPConditionStatus("ObjectFound");
    }else{
        RoboBreizhManagerUtils::setPNPConditionStatus("ObjectNotFound");
    }
}
} // namespace plan
} // namespace vision
}// namespace robobreizh

