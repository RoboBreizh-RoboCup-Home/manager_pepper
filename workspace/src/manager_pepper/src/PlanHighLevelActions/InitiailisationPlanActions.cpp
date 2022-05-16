#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Pose.h"
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"
#include "MongoDbUtils.hpp"

#include <sstream>
#include <cassert>

using namespace geometry_msgs;
using namespace mongodb_store;
using namespace std;

namespace robobreizh
{
namespace initialisation
{
namespace plan
{

void aInitCarryMyLuggage (string params, bool* run)
{
    ROS_INFO("1.1 Carry My Luggage - initialisation done");
    *run = 1;
}

void aInitGPSR(string params, bool* run)
{
    // TODO: Add global variables initiailisation here
    ROS_INFO("1.5 General Purpose Service Robot - initialisation");
    std_msgs::Int32 param_current_order;
    param_current_order.data = 2;
    MongoDbUtils::storeNewParameter<std_msgs::Int32>("param_gpsr_i_current_order", param_current_order);
    ROS_INFO("Okay, all good now :)");

    bool is_value_available;
    std_msgs::Int32 value = MongoDbUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_current_order", is_value_available);
    ROS_INFO("aInitGPSR - Value acquired");

    if (is_value_available)
    {   
        int value_converted = value.data;
        ROS_INFO("aInitGPSR - Value of %s : %d","param_gpsr_i_current_order" , value_converted);
        cout << "Value of param_gpsr_i_current_order: " << value.data << endl;
    }
        
    else
        ROS_INFO("aInitGPSR - No value found for %s", "param_gpsr_i_current_order");
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
