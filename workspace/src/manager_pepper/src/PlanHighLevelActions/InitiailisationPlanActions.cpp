#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <perception_pepper/Object.h>

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

    // Initialise parameters

    // i_current_order: int - Initialised to 0
    std_msgs::Int32 param_i_current_order;
    param_i_current_order.data = 0;
    MongoDbUtils::storeNewParameter<std_msgs::Int32>("param_gpsr_i_current_order", param_i_current_order);

    // current_order_type: String  -  Type used by Intent NLP => Used by the manager to choose tree branch of type of current order
    std_msgs::String param_current_order_type;
    param_current_order_type.data = string("STOP");
    MongoDbUtils::storeNewParameter<std_msgs::String>("param_gpsr_current_order_type", param_current_order_type);

    // number_of_orders: int - Number of orders contained in order list => len(commands)
    std_msgs::Int32 param_number_of_orders;
    param_number_of_orders.data = 0;
    MongoDbUtils::storeNewParameter<std_msgs::Int32>("param_gpsr_number_of_orders", param_number_of_orders);

    // Not supposed to be here: add object to list
    perception_pepper::Object obj;

    std_msgs::String obj_label;
    obj_label.data = "Roy Batty";

    obj.label = obj_label;
    obj.coord.x = 2.3;
    obj.coord.y = 1.3;
    obj.coord.z = 2.2;
    obj.distance = 1.1;
    obj.score = 0.6;
    MongoDbUtils::storeNewParameter<perception_pepper::Object>("param_obj_test", obj);

    // Example of variables reading
    bool is_value_available;
    std_msgs::Int32 value = MongoDbUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_current_order", is_value_available);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value of %s : %d","param_gpsr_i_current_order" , value.data);
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", "param_gpsr_i_current_order");

    /*
    std_msgs::String value_current_order = MongoDbUtils::getParameterValue<std_msgs::String>("param_gpsr_current_order_type", is_value_available);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value of %s : %s", "param_gpsr_current_order_type" , value_current_order.data.c_str());
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", "param_gpsr_current_order_type");
    */
    perception_pepper::Object obj_value = MongoDbUtils::getParameterValue<perception_pepper::Object>("param_obj_test", is_value_available);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value found at %s" , "param_obj_test");
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", "param_obj_test");
     // END Example of variables reading
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
