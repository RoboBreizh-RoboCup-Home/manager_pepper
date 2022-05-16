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
    string param_i_current_order_name = "param_gpsr_i_current_order";
    std_msgs::Int32 param_i_current_order;
    param_i_current_order.data = 0;
    MongoDbUtils::storeNewParameter<std_msgs::Int32>(param_i_current_order_name, param_i_current_order);

    // current_order_type: String  -  Type used by Intent NLP => Used by the manager to choose tree branch of type of current order
    string param_current_order_type_name;
    std_msgs::String param_current_order_type;
    param_current_order_type.data = string("STOP");
    MongoDbUtils::storeNewParameter<std_msgs::String>(param_current_order_type_name, param_current_order_type);

    // number_of_orders: int - Number of orders contained in order list => len(commands)
    string param_number_of_orders_name;
    std_msgs::Int32 param_number_of_orders;
    param_number_of_orders.data = 0;
    MongoDbUtils::storeNewParameter<std_msgs::Int32>(param_number_of_orders_name, param_number_of_orders);

    // Not supposed to be here: add object to list
    string obj_name_db = "param_obj_test";
    perception_pepper::Object obj;

    std_msgs::String obj_label;
    string obj_name = "Roy Batty";
    obj_label.data = obj_name;

    obj.label = obj_label;
    obj.coord.x = 2.3;
    obj.coord.y = 1.3;
    obj.coord.z = 2.2;
    obj.distance = 1.1;
    obj.score = 0.6;
    MongoDbUtils::storeNewParameter<perception_pepper::Object>(obj_name_db, obj);

    // Example of variables reading
    bool is_value_available;
    std_msgs::Int32 value = MongoDbUtils::getParameterValue<std_msgs::Int32>(param_i_current_order_name, is_value_available);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value of %s : %d",param_i_current_order_name.c_str() , value.data);
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", param_i_current_order_name.c_str());

    
    std_msgs::String value_current_order = MongoDbUtils::getParameterValue<std_msgs::String>(param_i_current_order_name.c_str(), is_value_available);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value of %s : %s", param_i_current_order_name.c_str() , value_current_order.data.c_str());
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", param_i_current_order_name.c_str());
    
    perception_pepper::Object obj_value = MongoDbUtils::getParameterValue<perception_pepper::Object>(obj_name_db, is_value_available);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value found at %s" , obj_name_db.c_str());
        ROS_INFO("aInitGPSR - label = %s", obj_value.label.data.c_str());
        ROS_INFO("aInitGPSR - obj.coord.x = %lf obj.coord.y = %lf obj.coord.z = %lf", obj_value.coord.x, obj_value.coord.y, obj_value.coord.z);
        ROS_INFO("aInitGPSR - obj.distance = %lf obj.score = %lf", obj_value.distance, obj_value.score);
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", obj_name_db.c_str());
     // END Example of variables reading*/
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
