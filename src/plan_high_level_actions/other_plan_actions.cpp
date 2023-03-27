#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <pnp_ros/names.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

#include "plan_high_level_actions/other_plan_actions.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"
#include "plan_high_level_actions/initialisation_plan_actions.hpp"
#include "database_model/person_model.hpp"
#include "database_model/gpsr_actions_model.hpp"

using namespace std;
using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

namespace robobreizh {
namespace other {
namespace plan {
void aGPSRProcessOrders(string params, bool* run) {
  string pnpNextAction;
  database::GPSRActionsModel gpsrActionDb;

  // Get total number of actions
  ROS_INFO("aGPSRProcessOrders - number of actions to execute = %d", g_nb_action);

  // Increment action id
  g_order_index++;

  RoboBreizhManagerUtils::setPNPConditionStatus("nextOrderNotKnownYet");
  if (g_order_index <= g_nb_action) {
    // Get Next Action info
    int currentStep = g_order_index;
    database::GPSRAction gpsrAction = gpsrActionDb.getAction(currentStep);
    ROS_INFO("aGPSRProcessOrders - intent = %s", gpsrAction.intent.c_str());

    if (gpsrAction.intent == "take") {
      if (!gpsrAction.object_item.empty()) {
        if (!gpsrAction.destination.empty() || !gpsrAction.source.empty()) {
          ROS_INFO("[ProcessOrder][take] item: %s, dest: %s, sour: %s -> NextOrderTakeObject",
                   gpsrAction.object_item.c_str(), gpsrAction.destination.c_str(), gpsrAction.source.c_str());
          pnpNextAction = "nextOrderTakeObject";
        } else {
          ROS_WARN("No destination or source found for the take intent");
        }
      } else {
        ROS_WARN("No object found for the take intent");
        // pnpNextAction = "nextOrderEscortHuman";
      }

      // ROS_INFO("intent = %s , object = %s , destination = %s, person = %s", gpsrAction.intent.c_str(),
      //          gpsrAction.object_item.c_str(), gpsrAction.destination.c_str(), gpsrAction.person.c_str());
    }

    else if (gpsrAction.intent == "go") {
      pnpNextAction = "nextOrderSTOP";
      ROS_ERROR("This has not been tested or implemented yet");
      // pnpNextAction = "nextOrderMoveTowards";
      // ROS_INFO("intent = %s , destination = %s", gpsrAction.intent.c_str(), gpsrAction.destination.c_str());
    }

    else if (gpsrAction.intent == "greet") {
      pnpNextAction = "nextOrderSTOP";
      ROS_ERROR("This has not been tested or implemented yet");
      // pnpNextAction = "nextOrderMoveTowards";
      // ROS_INFO("intent = %s , destination = %s", gpsrAction.intent.c_str(), gpsrAction.destination.c_str());
    } else if (gpsrAction.intent == "guide") {
      pnpNextAction = "nextOrderSTOP";
      ROS_ERROR("This has not been tested or implemented yet");
      // pnpNextAction = "nextOrderMoveTowards";
      // ROS_INFO("intent = %s , destination = %s", gpsrAction.intent.c_str(), gpsrAction.destination.c_str());
    } else if (gpsrAction.intent == "know") {
      pnpNextAction = "nextOrderSTOP";
      ROS_ERROR("This has not been tested or implemented yet");
      // pnpNextAction = "nextOrderMoveTowards";
      // ROS_INFO("intent = %s , destination = %s", gpsrAction.intent.c_str(), gpsrAction.destination.c_str());
    } else if (gpsrAction.intent == "follow") {
      pnpNextAction = "nextOrderSTOP";
      ROS_ERROR("This has not been tested or implemented yet");
      // pnpNextAction = "nextOrderFollowHuman";
      // ROS_INFO("intent = %s , person = %s", gpsrAction.intent.c_str(), gpsrAction.person.c_str());
    } else if (gpsrAction.intent == "find") {
      pnpNextAction = "nextOrderSTOP";
      ROS_ERROR("This has not been tested or implemented yet");
      // pnpNextAction = "nextOrderMoveTowards";
      // ROS_INFO("intent = %s , destination = %s", gpsrAction.intent.c_str(), gpsrAction.destination.c_str());
    }

    // else if (gpsrAction.intent == "to find something") {
    //   pnpNextAction = "nextOrderFindObject";
    //   ROS_INFO("intent = %s , object = %s , destination = %s, person = %s", gpsrAction.intent.c_str(),
    //            gpsrAction.object_item.c_str(), gpsrAction.destination.c_str(), gpsrAction.person.c_str());
    // }

    // else if (gpsrAction.intent == "to find someone") {
    //   pnpNextAction = "nextOrderFindHuman";
    //   ROS_INFO("intent = %s , object = %s , destination = %s, person = %s", gpsrAction.intent.c_str(),
    //            gpsrAction.object_item.c_str(), gpsrAction.destination.c_str(), gpsrAction.person.c_str());
    // }

    // else if (gpsrAction.intent == "say") {
    //   ROS_INFO("intent = %s , what = %s , who = %s", gpsrAction.intent.c_str(), gpsrAction.what.c_str(),
    //            gpsrAction.who.c_str());
    //   pnpNextAction = "nextOrderTell";
    // }
  }

  else
    pnpNextAction = "nextOrderSTOP";

  ROS_INFO("PnpNextAction = %s", pnpNextAction.c_str());
  RoboBreizhManagerUtils::setPNPConditionStatus(pnpNextAction);
  *run = 1;
}

void aIsHumanKnown(string params, bool* run) {
  string humanName;

  if (params == "GPSR") {
    GPSRActionsModel gpsrActionsDb;
    humanName = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
  } else
    humanName = params;

  // Access Database to find if wether or not the target is in the database

  RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotKnown");
  // If known
  // RoboBreizhManagerUtils::setPNPConditionStatus("HumanKnown");
}

// reuse of previous stuff for something not appropriate
void aCheckForMoreObjectTofind(string params, bool* run) {
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

  if (number_guests_to_welcome.data > number_guests_welcomed.data) {
    RoboBreizhManagerUtils::setPNPConditionStatus("MoreObjectToFind");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("NoMoreObjectToFind");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  }
  *run = 1;
}

void aCheckForMoreGuests(string params, bool* run) {
#ifdef LEGACY
  if (params == "fmm") {
    robobreizh::database::PersonModel vm;
    int numberOfPerson = vm.getPersons().size();
    std::cout << std::to_string(numberOfPerson) << std::endl;

    if (numberOfPerson < 4) {
      RoboBreizhManagerUtils::setPNPConditionStatus("MoreGuestToWelcome");
    } else {
      RoboBreizhManagerUtils::setPNPConditionStatus("NoMoreGuestToWelcome");
      RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    }
    *run = 1;
    return;
  }
#endif

  g_guest_counter++;

  if (g_guest_counter > g_guest_limit) {
    ROS_INFO("aCheckForMoreGuests - Welcomed %d/%d person ", g_guest_counter, g_guest_limit);
    RoboBreizhManagerUtils::setPNPConditionStatus("MoreGuestToWelcome");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(3);
  } else {
    ROS_WARN("aCheckForMoreGuests - Welcomed %d/%d person ", g_guest_counter, g_guest_limit);
    RoboBreizhManagerUtils::setPNPConditionStatus("NoMoreGuestToWelcome");
  }
  *run = 1;
}

void aChangePlan(string params, bool* run) {
  string pnpPlanTopicName = "/pnp/planToExec";
  std_msgs::String planNameMsg;
  planNameMsg.data = params;
  RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(pnpPlanTopicName, planNameMsg);
}

void aWait(string params, bool* run) {
  float sleepDuration = stoi(params);
  ros::Duration(sleepDuration).sleep();
  *run = 1;
}

void aChooseTake(std::string params, bool* run) {
  GPSRActionsModel gpsrActionsDb;
  auto gpsr_action = gpsrActionsDb.getAction(g_order_index);
  if (!gpsr_action.source.empty() && !gpsr_action.destination.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("SourceDestination");
  } else if (!gpsr_action.source.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("Source");
  } else if (!gpsr_action.destination.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("Destination");
  } else {
    ROS_ERROR("[aChoostake] - A case was not handled");
  }
  *run = 1;
}
}  // namespace plan
}  // namespace other
}  // namespace robobreizh
