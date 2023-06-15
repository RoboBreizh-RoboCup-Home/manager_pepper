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
// check whether there's person or object in the destination
void aCheckObjectAndHuman(string params, bool* run) {
  string pnpNextAction;
  database::GPSRActionsModel gpsrActionDb;

  database::GPSRAction gpsrAction = gpsrActionDb.getAction(1);
  ROS_INFO("aCheckObjectAndHuman - intent = %s", gpsrAction.intent.c_str());

  if (!gpsrAction.object_item.item_context.empty()) {
    ROS_INFO("[ProcessOrder][find] Object: %s, dest: %s -> nextOrderFindObject", gpsrAction.object_item.item_context.c_str(),
             gpsrAction.destination.item_context.c_str());
    pnpNextAction = "nextOrderFindObject";
  } else if (!gpsrAction.person.item_context.empty()) {
    ROS_INFO("[ProcessOrder][find] person: %s, dest: %s-> nextOrderFindHuman", gpsrAction.person.item_context.c_str(),
             gpsrAction.destination.item_context.c_str());
    pnpNextAction = "nextOrderFindHuman";
  } else {
    ROS_WARN("No Object found after moving to destination");
    pnpNextAction = "askNewInstruction";
  }
  ROS_INFO("PnpNextAction = %s", pnpNextAction.c_str());
  RoboBreizhManagerUtils::setPNPConditionStatus(pnpNextAction);
  *run = 1;
}

void aGPSRProcessOrders(string params, bool* run) {
  string pnpNextAction;
  database::GPSRActionsModel gpsrActionDb;

  // Get total number of actionss
  ROS_INFO("aGPSRProcessOrders - number of actions to execute = %d", g_nb_action);
  ROS_INFO("aGPSRProcessOrders - g_order_index= %d", g_order_index);
  // Increment action id
  g_order_index++;

  RoboBreizhManagerUtils::setPNPConditionStatus("nextOrderNotKnownYet");
  if (g_order_index <= g_nb_action) {
    // Get Next Action infoe
    int currentStep = g_order_index;
    database::GPSRAction gpsrAction = gpsrActionDb.getAction(currentStep);
    ROS_INFO("aGPSRProcessOrders - intent = %s", gpsrAction.intent.c_str());

    if (gpsrAction.intent == "take") {
      if (!gpsrAction.object_item.item_context.empty()) {
        if (!gpsrAction.destination.item_context.empty() || !gpsrAction.source.item_context.empty()) {
          ROS_INFO("[ProcessOrder][take] item: %s, dest: %s, sour: %s -> NextOrderTakeObject",
                   gpsrAction.object_item.item_context.c_str(), gpsrAction.destination.item_context.c_str(), gpsrAction.source.item_context.c_str());
          pnpNextAction = "nextOrderTakeObject";
        } else {
          ROS_WARN("No destination or source found for the take intent");
          pnpNextAction = "nextOrderSTOP";
        }
      } else {
        ROS_WARN("No object found for the take intent");
        pnpNextAction = "nextOrderSTOP";
      }
    } else if (gpsrAction.intent == "go") {
      if (!gpsrAction.destination.item_context.empty()) {
        pnpNextAction = "nextOrderMoveTowards";
      } else {
        ROS_WARN("No destination was found for the go intent");
        pnpNextAction = "nextOrderSTOP";
      }
    } else if (gpsrAction.intent == "greet") {
      if (!gpsrAction.person.item_context.empty()) {
        pnpNextAction = "nextOrderGreet";
      } else {
        pnpNextAction = "nextOrderSTOP";
      }
    } else if (gpsrAction.intent == "guide") {
      if (!gpsrAction.destination.item_context.empty()) {
        pnpNextAction = "nextOrderGuide";
      } else {
        ROS_WARN("No destination was found for the guide intent");
        pnpNextAction = "nextOrderSTOP";
      }
    } else if (gpsrAction.intent == "know") {
      pnpNextAction = "nextOrderSTOP";
      ROS_ERROR("This has not been tested or implemented yet");
    } else if (gpsrAction.intent == "follow") {
      if (!gpsrAction.person.item_context.empty()) {
        pnpNextAction = "nextOrderFollowHuman";
      } else {
        ROS_WARN("No person was found for the follow intent");
        pnpNextAction = "nextOrderSTOP";
      }
    } else if (gpsrAction.intent == "find") {
      // move to destination first and check whether there's human or object
      if (!gpsrAction.destination.item_context.empty()) {
        ROS_INFO("[ProcessOrder][move] destination: %s -> nextOrderMoveTowards", gpsrAction.destination.item_context.c_str());
        pnpNextAction = "nextOrderMoveTowards";
        // when it's already at the destination
      } else if (gpsrAction.destination.item_context.empty()) {
        if (!gpsrAction.person.item_context.empty()) {
          ROS_INFO("[ProcessOrder][find] Human: %s -> nextOrderFindHuman", gpsrAction.person.item_context.c_str());
          pnpNextAction = "nextOrderFindHuman";
        } else {
          ROS_INFO("[ProcessOrder][find] Object: %s -> nextOrderFindObject", gpsrAction.object_item.item_context.c_str());
          pnpNextAction = "nextOrderFindObject";
        }
      } else {
        ROS_WARN("No person was found for the find intent");
        pnpNextAction = "nextOrderSTOP";
      }
    }
  } else {
    pnpNextAction = "nextOrderSTOP";
  }

  ROS_INFO("PnpNextAction = %s", pnpNextAction.c_str());
  RoboBreizhManagerUtils::setPNPConditionStatus(pnpNextAction);
  *run = 1;
}

void aIsHumanKnown(string params, bool* run) {
  robobreizh::database::PersonModel pm;
  std::string humanName;

  if (params == "GPSR") {
    GPSRActionsModel gpsrActionsDb;
    humanName = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person_id);
  } else
    humanName = params;

  // Access Database to find if wether or not the target is in the database
  if (!pm.getPersonByName(humanName).name.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("HumanKnown");
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("HumanNotKnown");
  }
}

// reuse of previous stuff for something not appropriate
void aCheckForMoreObjectTofind(string params, bool* run) {
  // NOT SUPPOSED TO BE HERE - Increment number_guests_welcomed
  g_guest_counter++;

  ROS_INFO("aCheckForMoreGuests - Number of guests to welcome = %d", g_guest_limit);
  ROS_INFO("aCheckForMoreGuests - Number of guests welcomed = %d", g_guest_counter);

  if (g_guest_limit > g_guest_counter) {
    RoboBreizhManagerUtils::setPNPConditionStatus("MoreObjectToFind");
  } else {
    RoboBreizhManagerUtils::setPNPConditionStatus("NoMoreObjectToFind");
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
    }
    *run = 1;
    return;
  }
#endif

  g_guest_counter++;
  std_msgs::Int32 g_guest_limit;
  SQLiteUtils::getParameterValue("guest_limit", g_guest_limit);
  std::cout << "guest_count" << g_guest_counter << "g_guest_limit = " << g_guest_limit.data << std::endl;
  if (g_guest_counter < g_guest_limit.data) {
    ROS_INFO("aCheckForMoreGuests - Welcomed %d/%d person ", g_guest_counter, g_guest_limit.data);
    RoboBreizhManagerUtils::setPNPConditionStatus("MoreGuestToWelcome");
  } else {
    ROS_WARN("aCheckForMoreGuests - Welcomed %d/%d person ", g_guest_counter, g_guest_limit.data);
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
  if (!gpsr_action.source.item_context.empty() && !gpsr_action.destination.item_context.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("SourceDestination");
  } else if (!gpsr_action.source.item_context.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("Source");
  } else if (!gpsr_action.destination.item_context.empty()) {
    RoboBreizhManagerUtils::setPNPConditionStatus("Destination");
  } else {
    ROS_ERROR("[aChoostake] - A case was not handled");
  }
  *run = 1;
}

/**
 * @brief Update the person that broke the rule and set it as solved
 */
void aSticklerUpdateFinished(std::string params, bool* run) {
  std_msgs::Int32 stickler_tracked_person;
  if (!SQLiteUtils::getParameterValue<std_msgs::Int32>("stickler_tracker_person_name", stickler_tracked_person)) {
    ROS_ERROR_STREAM("Error while getting stickler_tracked_person");
    ROS_ERROR_STREAM("This should not happen");
    return;
  }
  robobreizh::database::PersonModel pm;
  auto person = pm.getPerson(stickler_tracked_person.data);
  if (params == "ForbiddenRoom") {
    // delete the person from the db as the position is now wrong
    pm.deletePerson(person.id);
  } else if (params == "Drink") {
    person.is_drink = true;
    pm.updatePerson(stickler_tracked_person.data, person);
  } else if (params == "Shoes") {
    person.is_shoes = true;
    pm.updatePerson(stickler_tracked_person.data, person);
  }
  *run = 1;
}

}  // namespace plan
}  // namespace other
}  // namespace robobreizh
