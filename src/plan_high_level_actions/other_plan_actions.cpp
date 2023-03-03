#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <pnp_ros/names.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

#include "plan_high_level_actions/other_plan_actions.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"
#include "database_model/person_model.hpp"
#include "database_model/gpsr_actions_model.hpp"

using namespace std;
using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

namespace robobreizh
{
namespace other
{
namespace plan
{
void aGPSRProcessOrders(string params, bool* run)
{
  string pnpNextAction;
  database::GPSRActionsModel gpsrActionDb;

  // Get total number of actions
  std_msgs::Int32 number_actions;
  bool ret = SQLiteUtils::getParameterValue<std_msgs::Int32>("param_gpsr_nb_actions", number_actions);
  ROS_INFO("aGPSRProcessOrders - param_gpsr_nb_actions = %d", number_actions.data);

  // Get current action id
  bool is_value_available = false;
  std_msgs::Int32 current_action_id_int32;
  is_value_available = SQLiteUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

  // Increment action id
  current_action_id_int32.data++;
  ret = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

  RoboBreizhManagerUtils::setPNPConditionStatus("nextOrderNotKnownYet");
  if (current_action_id_int32.data <= number_actions.data)
  {
    // Get Next Action info
    int currentStep = current_action_id_int32.data;
    database::GPSRAction gpsrAction = gpsrActionDb.getAction(currentStep);
    ROS_INFO("aGPSRProcessOrders - intent = %s", gpsrAction.intent.c_str());

    if (gpsrAction.intent == "take")
    {
      if (!gpsrAction.object_item.empty())
        pnpNextAction = "nextOrderTakeObject";
      else
        pnpNextAction = "nextOrderEscortHuman";

      ROS_INFO("intent = %s , object = %s , destination = %s, person = %s", gpsrAction.intent.c_str(),
               gpsrAction.object_item.c_str(), gpsrAction.destination.c_str(), gpsrAction.person.c_str());
    }

    else if (gpsrAction.intent == "go")
    {
      pnpNextAction = "nextOrderMoveTowards";
      ROS_INFO("intent = %s , destination = %s", gpsrAction.intent.c_str(), gpsrAction.destination.c_str());
    }

    else if (gpsrAction.intent == "follow")
    {
      pnpNextAction = "nextOrderFollowHuman";
      ROS_INFO("intent = %s , person = %s", gpsrAction.intent.c_str(), gpsrAction.person.c_str());
    }

    else if (gpsrAction.intent == "to find something")
    {
      pnpNextAction = "nextOrderFindObject";
      ROS_INFO("intent = %s , object = %s , destination = %s, person = %s", gpsrAction.intent.c_str(),
               gpsrAction.object_item.c_str(), gpsrAction.destination.c_str(), gpsrAction.person.c_str());
    }

    else if (gpsrAction.intent == "to find someone")
    {
      pnpNextAction = "nextOrderFindHuman";
      ROS_INFO("intent = %s , object = %s , destination = %s, person = %s", gpsrAction.intent.c_str(),
               gpsrAction.object_item.c_str(), gpsrAction.destination.c_str(), gpsrAction.person.c_str());
    }

    else if (gpsrAction.intent == "say")
    {
      ROS_INFO("intent = %s , what = %s , who = %s", gpsrAction.intent.c_str(), gpsrAction.what.c_str(),
               gpsrAction.who.c_str());
      pnpNextAction = "nextOrderTell";
    }
  }

  else
    pnpNextAction = "nextOrderSTOP";

  ROS_INFO("PnpNextAction = %s", pnpNextAction.c_str());
  RoboBreizhManagerUtils::setPNPConditionStatus(pnpNextAction);
  *run = 1;
}

void aIsHumanKnown(string params, bool* run)
{
  string humanName;

  if (params == "GPSR")
  {
    GPSRActionsModel gpsrActionsDb;
    humanName = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
  }
  else
    humanName = params;

  // Access Database to find if wether or not the target is in the database

  RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotKnown");
  // If known
  // RoboBreizhManagerUtils::setPNPConditionStatus("HumanKnown");
}

// reuse of previous stuff for something not appropriate
void aCheckForMoreObjectTofind(string params, bool* run)
{
  std_msgs::Int32 number_guests_to_welcome, number_guests_welcomed;
  // NOT SUPPOSED TO BE HERE - Increment number_guests_welcomed
  SQLiteUtils::getParameterValue<std_msgs::Int32>("param_number_of_guests_welcomed", number_guests_welcomed);
  number_guests_welcomed.data++;
  bool ret =
      SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("param_number_of_guests_welcomed", number_guests_welcomed);
  // This is a temporary function, will be replaced for a more dynamic one
  bool is_value_available = false;

  SQLiteUtils::getParameterValue<std_msgs::Int32>("param_number_of_guests_to_welcome", number_guests_to_welcome);
  SQLiteUtils::getParameterValue<std_msgs::Int32>("param_number_of_guests_welcomed", number_guests_welcomed);

  ROS_INFO("aCheckForMoreGuests - Number of guests to welcome = %d", number_guests_to_welcome.data);
  ROS_INFO("aCheckForMoreGuests - Number of guests welcomed = %d", number_guests_welcomed.data);

  if (number_guests_to_welcome.data > number_guests_welcomed.data)
  {
    RoboBreizhManagerUtils::setPNPConditionStatus("MoreObjectToFind");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  }
  else
  {
    RoboBreizhManagerUtils::setPNPConditionStatus("NoMoreObjectToFind");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  }
  *run = 1;
}

void aCheckForMoreGuests(string params, bool* run)
{
  #ifdef LEGACY
  if (params == "fmm")
  {
    robobreizh::database::PersonModel vm;
    int numberOfPerson = vm.getPersons().size();
    std::cout << std::to_string(numberOfPerson) << std::endl;

    if (numberOfPerson < 4)
    {
      RoboBreizhManagerUtils::setPNPConditionStatus("MoreGuestToWelcome");
    }
    else
    {
      RoboBreizhManagerUtils::setPNPConditionStatus("NoMoreGuestToWelcome");
      RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    }
    *run = 1;
    return;
  }
  #endif

  std_msgs::Int32 number_guests_to_welcome, number_guests_welcomed;
  // NOT SUPPOSED TO BE HERE - Increment number_guests_welcomed
  SQLiteUtils::getParameterValue<std_msgs::Int32>("param_number_of_guests_welcomed", number_guests_welcomed);
  std::cout << "number of guests welcomed : " << number_guests_welcomed.data << std::endl;
  number_guests_welcomed.data++;
  std::cout << "number of guests welcomed : " << number_guests_welcomed.data << std::endl;
  bool ret = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("param_number_of_guests_welcomed", number_guests_welcomed);
  // This is a temporary function, will be replaced for a more dynamic one
  bool is_value_available = false;

  SQLiteUtils::getParameterValue<std_msgs::Int32>("param_number_of_guests_to_welcome", number_guests_to_welcome);
  SQLiteUtils::getParameterValue<std_msgs::Int32>("param_number_of_guests_welcomed", number_guests_welcomed);

  ROS_INFO("aCheckForMoreGuests - Number of guests to welcome = %d", number_guests_to_welcome.data);
  ROS_INFO("aCheckForMoreGuests - Number of guests welcomed = %d", number_guests_welcomed.data);

  if (number_guests_to_welcome.data > number_guests_welcomed.data)
  {
    RoboBreizhManagerUtils::setPNPConditionStatus("MoreGuestToWelcome");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);
  }
  else
  {
    RoboBreizhManagerUtils::setPNPConditionStatus("NoMoreGuestToWelcome");
  }
  *run = 1;
}

void aChangePlan(string params, bool* run)
{
  string pnpPlanTopicName = "/pnp/planToExec";
  std_msgs::String planNameMsg;
  planNameMsg.data = params;
  RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(pnpPlanTopicName, planNameMsg);
}

void aWait(string params, bool* run)
{
  float sleepDuration = stoi(params);
  ros::Duration(sleepDuration).sleep();
  *run = 1;
}
}  // namespace plan
}  // namespace other
}  // namespace robobreizh
