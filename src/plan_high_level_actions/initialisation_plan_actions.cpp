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

#include "generic_actions/navigation_generic_actions.hpp"

#include "plan_high_level_actions/initialisation_plan_actions.hpp"
#include "database_model/person_model.hpp"
#include "database_model/object_model.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"
#include "generic_actions/navigation_generic_actions.hpp"

using namespace std;

namespace robobreizh
{
namespace initialisation
{
namespace plan
{
void aInitCarryMyLuggage(string params, bool* run)
{
  ROS_INFO("1.1 Carry My Luggage - initialisation done");
  RoboBreizhManagerUtils::setPNPConditionStatus("CarryMyLuggageInitDone");
  *run = 1;
}

void aInitFarewell(string params, bool* run)
{
  // Delete all person in the db
  robobreizh::database::PersonModel pm;
  pm.clearPerson();

  robobreizh::database::ObjectModel om;
  om.clearObjects();

  // Add variables
  bool ret;
  string name_number_of_guests_to_welcome = "param_number_of_guests_to_welcome";
  std_msgs::Int32 param_number_of_guests_to_welcome;
  param_number_of_guests_to_welcome.data = 2;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_guests_to_welcome,
                                                        param_number_of_guests_to_welcome);

  string name_number_of_guests_welcomed = "param_number_of_guests_welcomed";
  std_msgs::Int32 param_number_of_guests_welcomed;
  param_number_of_guests_welcomed.data = 0;
  ret =
      SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_guests_welcomed, param_number_of_guests_welcomed);

  // do story for rviz
  std::string title = "Farewell";
  std::vector<std::string> storyline{ "Wait for door to open",
                                      "Move towards arena",
                                      "Ask to wave hand",
                                      "Look for someone waving",
                                      "Move closer to the person waving",
                                      "Greet the person",
                                      "Ask them if they want to leave",
                                      "Ask human to follow",
                                      "Move towards coat stand",
                                      "Ask to take the coat",
                                      "Ask confirmation",
                                      "Go outside",
                                      "Ask human to follow",
                                      "Find cab driver",
                                      "Finish" };
  sendPlanVizbox(title, storyline);

  // reset steps
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);

  RoboBreizhManagerUtils::setPNPConditionStatus("InitFarewellDone");
  *run = 1;
}

void aInitGPSR(string params, bool* run)
{
  // Delete all person in the db
  robobreizh::database::PersonModel pm;
  pm.clearPerson();

  robobreizh::database::ObjectModel om;
  om.clearObjects();

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

  // current_order_type: String  -  Type used by Intent NLP => Used by the manager to choose tree branch of type of
  // current order
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

  std::string title = "GPSR";
  std::vector<std::string> storyline{ "Wait for door opening", "Navigation instruction point", "Find human",
                                      "Greet human", "Listen orders" };

  sendPlanVizbox(title, storyline);

  RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);

  RoboBreizhManagerUtils::setPNPConditionStatus("GPSRInitDone");
  *run = 1;
}

void sendPlanVizbox(std::string title, std::vector<std::string> storyline)
{
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
  robobreizh::database::PersonModel pm;
  pm.clearPerson();

  robobreizh::database::ObjectModel om;
  om.clearObjects();
  // Add the host name and drink
  robobreizh::database::Person person;
  person.name = "Charles";
  person.favorite_drink = "Milk";
  pm.insertPerson(person);

  std::string title = "Receptionist";
  std::vector<std::string> storyline{ "Navigate to the arena ",
                                      "Find a human",
                                      "Welcome the person",
                                      "Ask for a name",
                                      "Ask for a favorite drink",
                                      "Ask to follow to the living room",
                                      "Navigate towards liviging room",
                                      "Introduce guest to people in the living room",
                                      "Introduce people in the living room to the guest",
                                      "Find empty seat",
                                      "Offer seat",
                                      "Finish" };
  sendPlanVizbox(title, storyline);

  // reset steps
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);

  string name_number_of_guests_to_welcome = "param_number_of_guests_to_welcome";
  std_msgs::Int32 param_number_of_guests_to_welcome;
  param_number_of_guests_to_welcome.data = 3;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_guests_to_welcome,
                                                        param_number_of_guests_to_welcome);

  string name_number_of_guests_welcomed = "param_number_of_guests_welcomed";
  std_msgs::Int32 param_number_of_guests_welcomed,number_guests_welcomed;
  param_number_of_guests_welcomed.data = 0;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_guests_welcomed, param_number_of_guests_welcomed);
  SQLiteUtils::modifyParameterParameter<std_msgs::Int32>(name_number_of_guests_welcomed, param_number_of_guests_welcomed);
  SQLiteUtils::getParameterValue<std_msgs::Int32>("param_number_of_guests_welcomed", number_guests_welcomed);
  std::cout << "number of guests welcomed : " << number_guests_welcomed.data << std::endl;

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
  robobreizh::database::PersonModel pm;
  pm.clearPerson();

  robobreizh::database::ObjectModel om;
  om.clearObjects();

  std::string title = "Find my mate";
  std::vector<std::string> storyline{ "Navigate to the arena ",
                                      "Find Human",
                                      "Ask to start the task",
                                      "Navigate to the living room",
                                      "Find Humans in the room and store information",
                                      "Navigate towards arena",
                                      "Find Human",
                                      "Describe guests",
                                      "Finish" };
  sendPlanVizbox(title, storyline);

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

void aInitStoringGroceries(string params, bool* run)
{
  ROS_INFO("1.9 Groceries - initialisation");
  // Delete all person in the db
  robobreizh::database::PersonModel pm;
  pm.clearPerson();

  robobreizh::database::ObjectModel om;
  om.clearObjects();

  std::string title = "Storing groceries";
  std::vector<std::string> storyline{ "Wait for door opening",
                                      "Move to arena",
                                      "Move to the table",
                                      "Check if there are object left to grab",
                                      "Ask someone to grab an object and put it on the tablet",
                                      "Ask for confirmation",
                                      "Move to cabinet",
                                      "Ask to put the item on shelf",
                                      "Ask confirmation",
                                      "Move to arena",
                                      "Finish" };
  sendPlanVizbox(title, storyline);

  // reset steps
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);

  // init the number of objects to get
  bool ret = false;
  string name_number_of_guests_to_welcome = "param_number_of_guests_to_welcome";
  std_msgs::Int32 param_number_of_guests_to_welcome;
  param_number_of_guests_to_welcome.data = 5;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_guests_to_welcome,
                                                        param_number_of_guests_to_welcome);

  string name_number_of_guests_welcomed = "param_number_of_guests_welcomed";
  std_msgs::Int32 param_number_of_guests_welcomed;
  param_number_of_guests_welcomed.data = 0;
  ret =
      SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_guests_welcomed, param_number_of_guests_welcomed);

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
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_customers_to_serve,
                                                        param_number_of_customers_to_serve);

  string name_number_of_customers_served = "param_number_of_guests_welcomed";
  std_msgs::Int32 param_number_of_customers_served;
  param_number_of_customers_served.data = 0;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_number_of_customers_served,
                                                        param_number_of_customers_served);
  RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
  *run = 1;
}

void aInitStickler(string params, bool* run)
{
  // TODO: Add global variables initiailisation here
  ROS_INFO("2.8 Stickler For The Rules - initialisation");
  // Delete all person in the db
  robobreizh::database::PersonModel pm;
  pm.clearPerson();

  robobreizh::database::ObjectModel om;
  om.clearObjects();

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
  ret = SQLiteUtils::storeNewParameter<std_msgs::String>(name_whereisthis_starting_location,
                                                         param_whereisthis_starting_location);

  RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
  *run = 1;
}

void aInitServeBreakfast(string params, bool* run)
{
  ROS_INFO("1.8 Serve Breakfast - initialisation");

  // TODO: Add variables

  RoboBreizhManagerUtils::setPNPConditionStatus("initSBDone");
  *run = 1;
}

void aInitCleanTheTable(string params, bool* run)
{
  ROS_INFO("2.1 Clean The Table - Initialisation");
  bool ret;

  // Const_Tableware_items_number: int = 3
  const string name_const_Tableware_items_number = "Const_Tableware_items_number";
  std_msgs::Int32 const_Tableware_items_number;
  const_Tableware_items_number.data = 3;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_const_Tableware_items_number,
                                                         const_Tableware_items_number);
  // Const_Silverware items_number: int = 2
  const string name_const_Silverware_items_number = "Const_Silverware_items_number";
  std_msgs::Int32 const_Silverware_items_number;
  const_Silverware_items_number.data = 2;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_const_Silverware_items_number,
                                                         const_Silverware_items_number);

  // Param_Tableware_items_processed: int = 0
  const string name_param_Tableware_items_processed = "Param_Tableware_items_processed";
  std_msgs::Int32 param_Tableware_items_processed;
  param_Tableware_items_processed.data = 0;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_param_Tableware_items_processed,
                                                         param_Tableware_items_processed);

  // Param_Silverware_items_processed: int = 0
  const string name_param_Silverware_items_processed = "Param_Silverware_items_processed";
  std_msgs::Int32 param_Silverware_items_processed;
  param_Silverware_items_processed.data = 0;
  ret = SQLiteUtils::storeNewParameter<std_msgs::Int32>(name_param_Silverware_items_processed,
                                                         param_Silverware_items_processed);

  RoboBreizhManagerUtils::setPNPConditionStatus("initCTDone");
  *run = 1;
}
}  // namespace plan
}  // namespace initialisation
}  // namespace robobreizh
