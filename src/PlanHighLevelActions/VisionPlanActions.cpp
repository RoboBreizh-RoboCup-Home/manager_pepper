#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/VisionPlanActions.hpp"
#include "GenericActions/VisionGenericActions.hpp"
#include "DatabaseModel/VisionModel.hpp"
#include "ManagerUtils.hpp"

using namespace std;

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

    /*Basic movement - Point head to the ground (if object is for example on the ground) or around to find the object
    => Module not implemented yet"*/

    /* CV - Detect luggage */
    ROS_INFO("FindObject - Currently looking for %s", objectToFind.c_str());
    *run = vision::generic::findObject(objectToFind);
}

void aFindHuman(std::string params, bool* run)
{
    if (params.empty())
    {
        // Find any Human
        bool getHuman = false;
        do
        {
            getHuman = vision::generic::waitForHuman(); 
        } while (!getHuman); 
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
    system("rosservice call /robobreizh/manipulation/look_up");
    robobreizh::Person person;

    do
    {
        getHuman = vision::generic::findHumanAndStoreFeatures(&person); 
    } while (!getHuman); 

    RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
    system("rosservice call /robobreizh/manipulation/look_down");

    // format person in text
    RoboBreizhManagerUtils::pubVizBoxRobotText("gender : " + person.gender + ", age" + person.age + ", cloth color" + person.cloth_color + ", skin color : " + person.skin_color);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = 1;
}


void aFindHumanAndStoreFeaturesWithDistanceFilter(double param, bool* run)
{
    bool getHuman = false;
    system("rosservice call /robobreizh/manipulation/look_up");
    robobreizh::Person person;
    
    double distanceMax = param;

    do
    {
        getHuman = vision::generic::findHumanAndStoreFeaturesWithDistanceFilter(&person,distanceMax); 
    } while (!getHuman); 

    RoboBreizhManagerUtils::setPNPConditionStatus("GenderFound");
    system("rosservice call /robobreizh/manipulation/look_down");

    // format person in text
    RoboBreizhManagerUtils::pubVizBoxRobotText("gender : " + person.gender + ", age" + person.age + ", cloth color" + person.cloth_color + ", skin color : " + person.skin_color);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = 1;
}


void aFindEmptySeat(std::string params, bool* run){
    bool isFree = false;
    system("rosservice call manipulation_pepper /robobreizh/manipulation/look_down");
    do {
        isFree = vision::generic::FindEmptySeat(); 
    }while(!isFree);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::setPNPConditionStatus("EmptySeatFound");
    *run = 1;
}

} // namespace plan
} // namespace vision
}// namespace robobreizh
