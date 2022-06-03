#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <perception_pepper/Object.h>

#include <warehouse_ros_sqlite/database_connection.h>
#include <warehouse_ros_sqlite/utils.h>

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"
#include "SQLiteUtils.hpp"
#include "GenericActions/VisionGenericActions.hpp"

#include <sstream>
#include <cassert>

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

    // Test SQLite
    std::unique_ptr<warehouse_ros_sqlite::DatabaseConnection> conn_;
    ROS_INFO("1");
    conn_.reset(new warehouse_ros_sqlite::DatabaseConnection());
    ROS_INFO("2");
    conn_->setParams(":memory:", 0);
    ROS_INFO("3");
    bool ret = conn_->connect();
    ROS_INFO("4");

    if (ret)
        ROS_INFO("Connection done");
    else
        ROS_INFO("Connection failed");

    using V = geometry_msgs::Vector3;
    auto coll = conn_->openCollection<V>("main", "coll");
    auto meta1 = coll.createMetadata();
    meta1->append("x", 3);

    V v1, v2;
    v1.x = 3.0;
    v1.y = 1.0;

    coll.insert(v1, meta1);

    ROS_INFO("coll.count() = %u", coll.count()); // Expected result = 1

    v2.x = 5.0;
    v2.y = 7.0;

    auto meta2 = coll.createMetadata();
    meta2->append("x", 5);

    coll.insert(v2, meta2);

    ROS_INFO("coll.count() = %u", coll.count()); // Expected result = 2

    auto query = coll.createQuery();
    query->append("x", 3);

    const auto list = coll.queryList(query);

    ROS_INFO("List size for query x = 3 = %lu", list.size()); // Expected result = 1
    ROS_INFO("List size for query list[0] y value = %f", list[0]->y); // Expected result = 1.0

    coll.removeMessages(query);
    ROS_INFO("coll.count() = %u", coll.count()); // Expected result = 1

    
    // Initialise parameters
    // i_current_order: int - Initialised to 0
    /*string param_i_current_order_name = "param_gpsr_i_current_order";
    std_msgs::Int32 param_i_current_order;
    param_i_current_order.data = 0;
    bool ret;
    ret = MongoDbUtils::storeNewParameter<std_msgs::Int32>(param_i_current_order_name, param_i_current_order);

    // current_order_type: String  -  Type used by Intent NLP => Used by the manager to choose tree branch of type of current order
    string param_current_order_type_name = "param_gpsr_order_type";
    std_msgs::String param_current_order_type;
    param_current_order_type.data = string("STOP");
    ret = MongoDbUtils::storeNewParameter<std_msgs::String>(param_current_order_type_name, param_current_order_type);

    // number_of_orders: int - Number of orders contained in order list => len(commands)
    string param_number_of_orders_name;
    std_msgs::Int32 param_number_of_orders;
    param_number_of_orders.data = 0;
    ret = MongoDbUtils::storeNewParameter<std_msgs::Int32>(param_number_of_orders_name, param_number_of_orders);

    // Not supposed to be here: add object to list
    string obj_name_db = "param_obj_test";
    perception_pepper::Object obj;

    std_msgs::String obj_label;
    string obj_name = "Roy Batty";
    obj.label.data = obj_name;
    obj.coord.z = 2.2;
    obj.distance = 1.1;
    obj.score = 0.6;
    MongoDbUtils::storeNewParameter<perception_pepper::Object>(obj_name_db, obj);

    // Example of variables reading
    bool is_value_available = false;
    std_msgs::Int32 value;
    is_value_available = MongoDbUtils::getParameterValue<std_msgs::Int32>(param_i_current_order_name, value);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value of %s : %d",param_i_current_order_name.c_str() , value.data);
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", param_i_current_order_name.c_str());

    
    std_msgs::String value_current_order;
    is_value_available = MongoDbUtils::getParameterValue<std_msgs::String>(param_current_order_type_name, value_current_order);
    if (is_value_available)
        ROS_INFO("aInitGPSR - Value of %s : %s", param_current_order_type_name.c_str() , value_current_order.data.c_str());
    else
        ROS_INFO("aInitGPSR - No value found for %s", param_current_order_type_name.c_str());
    
    perception_pepper::Object obj_value;
    is_value_available = MongoDbUtils::getParameterValue<perception_pepper::Object>(obj_name_db, obj_value);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value found at %s" , obj_name_db.c_str());
        ROS_INFO("aInitGPSR - label = %s", obj_value.label.data.c_str());
        ROS_INFO("aInitGPSR - obj.coord.x = %lf obj.coord.y = %lf obj.coord.z = %lf", obj_value.coord.x, obj_value.coord.y, obj_value.coord.z);
        ROS_INFO("aInitGPSR - obj.distance = %lf obj.score = %lf", obj_value.distance, obj_value.score);
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", obj_name_db.c_str());

    // Check if some objects have already been found (not look for them again and save precious time)

    string objName_search = "Roy Batty";
    perception_pepper::Object obj_search_rb;
    is_value_available = vision::generic::getObjectIfAlreadyBeenFound(objName_search, obj_search_rb);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value found for object %s" , objName_search.c_str());
        ROS_INFO("aInitGPSR - label = %s", obj_search_rb.label.data.c_str());
        ROS_INFO("aInitGPSR - obj.coord.x = %lf obj.coord.y = %lf obj.coord.z = %lf", obj_search_rb.coord.x, obj_search_rb.coord.y, obj_search_rb.coord.z);
        ROS_INFO("aInitGPSR - obj.distance = %lf obj.score = %lf", obj_search_rb.distance, obj_search_rb.score);
    }
    else
        ROS_INFO("aInitGPSR - No value found for object %s", objName_search.c_str());

    objName_search = "spoon";
    perception_pepper::Object obj_search_spoon;
    is_value_available = vision::generic::getObjectIfAlreadyBeenFound(objName_search, obj_search_spoon);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Object %s found" , objName_search.c_str());
        ROS_INFO("aInitGPSR - label = %s", obj_search_spoon.label.data.c_str());
        ROS_INFO("aInitGPSR - obj.coord.x = %lf obj.coord.y = %lf obj.coord.z = %lf", obj_search_spoon.coord.x, obj_search_spoon.coord.y, obj_search_spoon.coord.z);
        ROS_INFO("aInitGPSR - obj.distance = %lf obj.score = %lf", obj_search_spoon.distance, obj_search_spoon.score);
    }
    else
        ROS_INFO("aInitGPSR - No object %s found", objName_search.c_str());
     // END Example of variables reading*/
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
