#include <std_msgs/String.h>
#include <ros/ros.h>

#include "PlanHighLevelActions/ManipulationPlanActions.hpp"
#include "GenericActions/ManipulationGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace manipulation
{
namespace plan
{

void aGrabHandle(std::string params, bool* run)
{
    // Get Parameters
    int i_object=params.find("_");
    int i_hand=params.find("_", i_object + 1);
    string object=params.substr(0, i_object);
    string hand=params.substr(i_object + 1, i_hand);

    ROS_INFO("aGrabHandle - grab object %s using %s hand", object.c_str(), hand.c_str());

    // CV - Find and localise handle 
    //Manipulation - Grasp handle  => Both actions should be on separate modules, with one to obtain object position, used in the following.
    manipulation::generic::grabHandle(object, hand);
    *run = 1;
}

void aDropObject(std::string params, bool* run)
{
    // Get Parameters
    string hand = params;

    ROS_INFO("aDropObject - drop object in %s hand", hand.c_str());

    // Manipulation - Put object held on a certain hand on a certain position
    manipulation::generic::dropObject(hand);
}

void aLook(std::string params, bool* run){
    
    ROS_INFO("looking " + params);
    if (params == "Up"){
        manipulation::generic::lookUp();
    } else if (params == "Down"){
        manipulation::generic::lookDown();
    } else if (params == "Around"){
        manipulation::generic::lookAround();
    } 
}

void aPointAt(std::string params,bool*run){

    ROS_INFO("point in front");
    manipulation::generic::pointInFront();
}
} // namespace plan
} // namespace manipulation
}// namespace robobreizh
