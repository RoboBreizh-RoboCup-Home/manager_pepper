#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <perception_pepper/Object.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <warehouse_ros_sqlite/database_connection.h>
#include <warehouse_ros_sqlite/utils.h>

#include "GenericActions/NavigationGenericActions.hpp"

#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"
#include "DatabaseModel/InitModel.hpp"
#include "DatabaseModel/VisionModel.hpp"
#include "ManagerUtils.hpp"
#include "SQLiteUtils.hpp"

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

void aInitFarewell(string params,bool*run){
    // Delete all person in the db
    robobreizh::database::InitModel im;
    im.deleteAllPerson();
    im.deleteAllObjects();

    // do story for rviz
    std::string title = "Farewell";
    std::vector<std::string> storyline;
    storyline.push_back("Wait for door to open");
    storyline.push_back("Move towards arena");
    storyline.push_back("Ask to wave hand");
    storyline.push_back("Look for someone waving");
    storyline.push_back("Move closer to the person waving");
    storyline.push_back("Greet the person");
    storyline.push_back("Ask them if they want to leave");
    storyline.push_back("Ask human to follow");
    storyline.push_back("Move towards coat stand");
    storyline.push_back("Ask to take the coat");
    storyline.push_back("Ask confirmation");
    storyline.push_back("Go outside");
    storyline.push_back("Ask human to follow");
    storyline.push_back("Find cab driver");
    storyline.push_back("Finish");
    sendPlanVizbox(title,storyline);

    // reset steps
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(3); 

    RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
    *run = 1;
}

void aInitGPSR(string params, bool* run)
{
    // TODO: Add global variables initiailisation here
    ROS_INFO("1.5 General Purpose Service Robot - initialisation");

    ROS_INFO("aInitGPSR - SQLite demonstration - START");
    // Initialise parameters
    bool ret = false;
    // i_current_order: int - Initialised to 0
    string param_i_current_order_name = "param_gpsr_i_action";
    std_msgs::Int32 param_i_current_order;
    param_i_current_order.data = 0;
    ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(param_i_current_order_name, param_i_current_order);

    // current_order_type: String  -  Type used by Intent NLP => Used by the manager to choose tree branch of type of current order
    string param_current_order_type_name = "param_gpsr_order_type";
    std_msgs::String param_current_order_type;
    param_current_order_type.data = string("STOP");
    ret = SQLiteUtils::storeNewParameter<std_msgs::String>(param_current_order_type_name, param_current_order_type);

    // number_of_orders: int - Number of orders contained in order list => len(commands)
    string param_number_of_orders_name = "param_gpsr_nb_actions";
    std_msgs::Int32 param_number_of_orders;
    param_number_of_orders.data = 0;
    ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(param_number_of_orders_name, param_number_of_orders);

    // Not supposed to be here: add object to list
    /*string obj_name_db = "param_obj_test";
    perception_pepper::Object obj;

    std_msgs::String obj_label;
    string obj_name = "fork";
    obj.label.data = obj_name;
    obj.coord.z = 2.2;
    obj.distance = 1.1;
    obj.score = 0.6;
    SQLiteUtils::storeNewParameter<perception_pepper::Object>(obj_name, obj);

    // Example of variables reading
    bool is_value_available = false;
    std_msgs::Int32 value;
    is_value_available = SQLiteUtils::getParameterValue<std_msgs::Int32>(param_i_current_order_name, value);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value of %s : %d",param_i_current_order_name.c_str() , value.data);
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", param_i_current_order_name.c_str());

    is_value_available = SQLiteUtils::getParameterValue<std_msgs::Int32>(param_number_of_orders_name, value);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value of %s : %d",param_number_of_orders_name.c_str() , value.data);
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", param_number_of_orders_name.c_str());
    

    
    std_msgs::String value_current_order;
    is_value_available = SQLiteUtils::getParameterValue<std_msgs::String>(param_current_order_type_name, value_current_order);
    if (is_value_available)
        ROS_INFO("aInitGPSR - Value of %s : %s", param_current_order_type_name.c_str() , value_current_order.data.c_str());
    else
        ROS_INFO("aInitGPSR - No value found for %s", param_current_order_type_name.c_str());
    
    // Check if some objects have already been found (not look for them again and save precious time)
    perception_pepper::Object obj_value;
    string objName_search = "spoon";
    is_value_available = SQLiteUtils::getParameterValue<perception_pepper::Object>(objName_search, obj_value);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Value found at %s" , obj_name_db.c_str());
        ROS_INFO("aInitGPSR - label = %s", obj_value.label.data.c_str());
        ROS_INFO("aInitGPSR - obj.coord.x = %lf obj.coord.y = %lf obj.coord.z = %lf", obj_value.coord.x, obj_value.coord.y, obj_value.coord.z);
        ROS_INFO("aInitGPSR - obj.distance = %lf obj.score = %lf", obj_value.distance, obj_value.score);
    }
    else
        ROS_INFO("aInitGPSR - No value found for %s", obj_name_db.c_str());

    objName_search = "fork";
    perception_pepper::Object obj_search_spoon;
    is_value_available = SQLiteUtils::getParameterValue(objName_search, obj_search_spoon);
    if (is_value_available)
    {   
        ROS_INFO("aInitGPSR - Object %s found" , objName_search.c_str());
        ROS_INFO("aInitGPSR - label = %s", obj_search_spoon.label.data.c_str());
        ROS_INFO("aInitGPSR - obj.coord.x = %lf obj.coord.y = %lf obj.coord.z = %lf", obj_search_spoon.coord.x, obj_search_spoon.coord.y, obj_search_spoon.coord.z);
        ROS_INFO("aInitGPSR - obj.distance = %lf obj.score = %lf", obj_search_spoon.distance, obj_search_spoon.score);
    }
    else
        ROS_INFO("aInitGPSR - No object named %s found", objName_search.c_str());
    ROS_INFO("aInitGPSR - SQLite demonstration - END");*/

    RoboBreizhManagerUtils::setPNPConditionStatus("GPSRInitDone");

    std::string title = "GPSR";
    std::vector<std::string> storyline;
    storyline.push_back("Wait for door opening");
    storyline.push_back("Navigation instruction point");
    storyline.push_back("Find human");
    storyline.push_back("Greet human");
    storyline.push_back("Listen orders");

    sendPlanVizbox(title,storyline);

    RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);

    *run = 1;
}

void sendPlanVizbox(std::string title, std::vector<std::string> storyline){
    vizbox::Story story;
    story.title = title;
    story.storyline = storyline;

    RoboBreizhManagerUtils::pubVizBoxStory(story);
}

void aInitReceptionist(string params, bool* run)
{
    ROS_INFO("1.6 Receptionist - initialisation");
    bool ret;

    // Delete all person in the db
    robobreizh::database::InitModel im;
    im.deleteAllSeatedPerson();
    im.deleteAllPerson();
    // Add the host name and drink
    std::cout<< std::endl<< std::endl<< std::endl<< std::endl;
    std::string hostName = "Charlie";
    std::string hostDrink= "Ice Tea";
    im.addReceptionistHost(hostName,hostDrink);

    std::string title = "Receptionist";
    std::vector<std::string> storyline;
    storyline.push_back("Navigate to the arena ");
    storyline.push_back("Find a human");
    storyline.push_back("Welcome the person");
    storyline.push_back("Ask for a name");
    storyline.push_back("Ask for a favorite drink");
    storyline.push_back("Ask to follow to the living room");
    storyline.push_back("Navigate towards liviging room");
    storyline.push_back("Introduce guest to people in the living room");
    storyline.push_back("Introduce people in the living room to the guest");
    storyline.push_back("Find empty seat");
    storyline.push_back("Offer seat");
    storyline.push_back("Finish");
    sendPlanVizbox(title,storyline);

    // reset steps
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);

    string name_number_of_guests_to_welcome = "param_number_of_guests_to_welcome";
    std_msgs::Int32 param_number_of_guests_to_welcome;
    param_number_of_guests_to_welcome.data = 2;
    ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_guests_to_welcome, param_number_of_guests_to_welcome);

    string name_number_of_guests_welcomed = "param_number_of_guests_welcomed";
    std_msgs::Int32 param_number_of_guests_welcomed;
    param_number_of_guests_welcomed.data = 0;
    ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_guests_welcomed, param_number_of_guests_welcomed);

    string nameNumberOfUnsuccessfulTries = "param_number_of_unsuccessful_tries";
    std_msgs::Int32 paramNumberOfUnsuccessfulTries;
    paramNumberOfUnsuccessfulTries.data = 0;
    ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(nameNumberOfUnsuccessfulTries, paramNumberOfUnsuccessfulTries);

    RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
    *run = 1;
}

void aInitFindMyMate(string params, bool* run)
{
    ROS_INFO("Find My Mate - initialisation");

    // Delete all person in the db
    robobreizh::database::InitModel im;
    im.deleteAllPerson();
    im.deleteAllObjects();
    /* im.deleteAllPersonRows(); */

    // insert operator 
    /*
    robobreizh::database::VisionModel vm;
    robobreizh::Person person;
    person.name = "operator";
    person.favorite_drink = "";
    person.gender = "";
    person.age = "";
    person.cloth_color = "";
    person.skin_color= "";
    person.posture = "";
    person.pos_x = 3.595;
    person.pos_y = 8.227;
    person.pos_z = 0.000;
    person.distance = 0.00; */

    // vm.createPerson(person);

    std::string title = "Find my mate";
    std::vector<std::string> storyline;
    storyline.push_back("Navigate to the arena ");
    storyline.push_back("Find Human");
    storyline.push_back("Ask to start the task");
    storyline.push_back("Navigate to the living room");
    storyline.push_back("Find Humans in the room and store information");
    storyline.push_back("Navigate towards arena");
    storyline.push_back("Find Human");
    storyline.push_back("Describe guests");
    storyline.push_back("Finish");
    sendPlanVizbox(title,storyline);

	geometry_msgs::PoseWithCovarianceStamped p;
	p.header.stamp = ros::Time::now();
	p.header.frame_id = "map";
	p.pose.pose.position.x = 2.709;
	p.pose.pose.position.y = 3.695;
	p.pose.pose.position.z = 0.000;

	p.pose.pose.orientation.x = 0.000;
	p.pose.pose.orientation.y = 0.000;
	p.pose.pose.orientation.z = 0.000;
	p.pose.pose.orientation.w = 0.000;

	navigation::generic::setInitPose(p);

    // reset steps
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(3); 

    RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
    *run = 1;
}

void aInitRestaurant(string params, bool* run)
{
    // TODO: Add global variables initiailisation here
    ROS_INFO("2.6 Restaurant - initialisation");

    // Initialise parameters
    bool ret = false;
    // We use the GPSR database to store orders, that's why variables are named the same
    // i_current_order: int - Initialised to 0
    string param_i_current_order_name = "param_gpsr_i_action";
    std_msgs::Int32 param_i_current_order;
    param_i_current_order.data = 0;
    ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(param_i_current_order_name, param_i_current_order);

    string name_number_of_customers_to_serve = "param_number_of_guests_to_welcome";
    std_msgs::Int32 param_number_of_customers_to_serve;
    param_number_of_customers_to_serve.data = 2;
    ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_customers_to_serve, param_number_of_customers_to_serve);

    string name_number_of_customers_served = "param_number_of_guests_welcomed";
    std_msgs::Int32 param_number_of_customers_served;
    param_number_of_customers_served.data = 0;
    ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_customers_served, param_number_of_customers_served);

    RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
    *run = 1;
}

void aInitStickler(string params, bool* run)
{
    // TODO: Add global variables initiailisation here
    ROS_INFO("2.8 Stickler For The Rules - initialisation");
    
    // Delete all person in the db
    robobreizh::database::InitModel im;
    im.deleteAllPerson();
    im.deleteAllObjects();

    RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
    *run = 1;
}

void aInitWhereIsThis(string params, bool* run)
{
    // TODO: Add global variables initiailisation here
    ROS_INFO("2.9 Where Is This? - initialisation");

    bool ret;

    const string name_whereisthis_furniture = "param_whereisthis_furniture";
    std_msgs::String param_whereisthis_furniture;
    param_whereisthis_furniture.data = "None";
    ret = SQLiteUtils::storeNewParameter<std_msgs::String>(name_whereisthis_furniture, param_whereisthis_furniture);

    const string name_whereisthis_starting_location = "param_whereisthis_starting_location";
    std_msgs::String param_whereisthis_starting_location;
    param_whereisthis_starting_location.data = "Office";
    ret = SQLiteUtils::storeNewParameter<std_msgs::String>(name_whereisthis_starting_location, param_whereisthis_starting_location);

    RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
    *run = 1;
}
} // namespace plan
} // namespace initialisation
} // namespace robobreizh
