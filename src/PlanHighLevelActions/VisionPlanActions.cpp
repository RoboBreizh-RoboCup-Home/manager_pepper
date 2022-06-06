#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/VisionPlanActions.hpp"
#include "GenericActions/VisionGenericActions.hpp"
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
    bool getHuman = false;
    do
    {
        getHuman = vision::generic::waitForHuman(); 
    } while (!getHuman); 
    RoboBreizhManagerUtils::setPNPConditionStatus("HFound");
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
    *run = 1;
}
} // namespace plan
} // namespace vision
}// namespace robobreizh