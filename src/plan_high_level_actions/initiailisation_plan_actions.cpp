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
#include "GenericActions/NavigationGenericActions.hpp"

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
  robobreizh::database::InitModel im;
  im.deleteAllPerson();
  im.deleteAllObjects();

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
  sendPlanVizbox(title, storyline);

  // reset steps
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);

  RoboBreizhManagerUtils::setPNPConditionStatus("InitFarewellDone");
  *run = 1;
}

void aInitGPSR(string params, bool* run)
{
  // Delete all person in the db
  robobreizh::database::InitModel im;
  im.deleteAllPerson();
  im.deleteAllObjects();

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
  std::vector<std::string> storyline;
  storyline.push_back("Wait for door opening");
  storyline.push_back("Navigation instruction point");
  storyline.push_back("Find human");
  storyline.push_back("Greet human");
  storyline.push_back("Listen orders");

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
  robobreizh::database::InitModel im;
  im.deleteAllSeatedPerson();
  im.deleteAllPerson();
  // Add the host name and drink
  std::cout << std::endl << std::endl << std::endl << std::endl;
  std::string hostName = "John";
  std::string hostDrink = "Milk";
  im.addReceptionistHost(hostName, hostDrink);

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
  sendPlanVizbox(title, storyline);

  // reset steps
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);

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
  robobreizh::database::InitModel im;
  im.deleteAllPerson();
  im.deleteAllObjects();

  std::string title = "Storing groceries";
  std::vector<std::string> storyline;
  storyline.push_back("Wait for door opening");
  storyline.push_back("Move to arena");
  storyline.push_back("Move to the table");
  storyline.push_back("Check if there are object left to grab");
  storyline.push_back("Ask someone to grab an object and put it on the tablet");
  storyline.push_back("Ask for confirmation");
  storyline.push_back("Move to cabinet");
  storyline.push_back("Ask to put the item on shelf");
  storyline.push_back("Ask confirmation");
  storyline.push_back("Move to arena");
  storyline.push_back("Finish");
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
  ret = SQLiteUtils::storeNewParameter<std_msgs::String>(name_whereisthis_starting_location,
                                                         param_whereisthis_starting_location);

  RoboBreizhManagerUtils::setPNPConditionStatus("InitDone");
  *run = 1;
}
}  // namespace plan
}  // namespace initialisation
}  // namespace robobreizh
