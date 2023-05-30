#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <boost/format.hpp>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pnp_ros/names.h>

#include "plan_high_level_actions/initialisation_plan_actions.hpp"
#include "plan_high_level_actions/dialog_plan_actions.hpp"
#include "generic_actions/dialog_generic_actions.hpp"
#include "generic_actions/navigation_generic_actions.hpp"
#include "generic_actions/gesture_generic_actions.hpp"
#include "vision_utils.hpp"
#include "database_model/person_model.hpp"
#include "database_model/location_model.hpp"
#include "database_model/object_model.hpp"
#include "database_model/speech_model.hpp"
#include "database_model/gpsr_actions_model.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"

using namespace std;

using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

namespace robobreizh {
namespace dialog {
namespace plan {

/**
 * @brief aSay takes a string as an argument from the petri net and say it using naoqi api.
 *
 * @param params string to say in camel case.
 * @param run boolean value allowing to change the state of the petri net.
 */
void aSay(string params, bool* run) {
  // split params into 2 strings using the _ symbol as a separator
  int i = params.find("_");
  std::string sentence = params.substr(0, i);
  std::string mode = params.substr(i + 1, params.length());
  int modeId;
  if (mode == "normal") {
    modeId = 1;
  } else if (mode == "animated") {
    modeId = 0;
  } else if (mode == "") {
    ROS_ERROR("[aSay] - mode not specified falling back on normal mode");
    modeId = 1;
  } else {
    ROS_ERROR("[aSay] - mode %s not supported falling back on normal mode", mode.c_str());
    modeId = 1;
  }
  std::string text = robobreizh::convertCamelCaseToSpacedText(sentence);
  *run = dialog::generic::robotSpeech(text, modeId);
}

/**
 * @brief ask human to place object last object.
 *
 * @param params
 * @param run
 */
void aDialogAskHumanPlaceLastObjectOnTablet(string params, bool* run) {
  robobreizh::database::ObjectModel om;
  robobreizh::database::Object obj = om.getLastObject();
  std::string text = "Could you please put the " + obj.label + " on the tablet";
  robobreizh::dialog::generic::robotSpeech(text, 1);
  ROS_INFO(text.c_str());
  *run = 1;
}

void aDialogAskHumanTakeLastObject(string params, bool* run) {
  robobreizh::database::ObjectModel om;
  robobreizh::database::Object obj = om.getLastObject();
  std::cout << obj.label << std::endl;
  std::string text = "Could you please take the " + obj.label + " with you.";
  robobreizh::dialog::generic::robotSpeech(text, 1);
  ROS_INFO(text.c_str());
  *run = 1;
}

void aAskHuman(string params, bool* run) {
  // Dialog - Text-To-Speech
  std::string action = robobreizh::convertCamelCaseToSpacedText(params);
  std::string textToPronounce =
      "Could you please indicate your " + action + ". Please make a sentence and say it loud and clear";

  // Specific cases
  if (params == "waveHandFarewell")
    textToPronounce = "Could you please wave your hands if you want to leave";

  if (params == "waveHand")
    textToPronounce = "I can't see you, could you please wave your hand";

  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce, 0);
}

void aAskHumanRepeat(string params, bool* run) {
  std::string textToPronounce = "Sorry, I didn't understand. Could you please repeat";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce, 0);
}

void aAskHumanToStartTask(string params, bool* run) {
  std::string textToPronounce = "To start the task please say : 'start the task'";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce, 0);
}

void aAskHumanToFollowToLocation(string params, bool* run) {
  // might need to split the string or something
  std::string action = robobreizh::convertCamelCaseToSpacedText(params);
  std::string textToPronounce = "Could you please follow me to the " + action;
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce, 1);
}

void aAskHumanToFollow(string params, bool* run) {
  std::string textToPronounce;
  if (params == "GPSR") {
    database::GPSRActionsModel gpsrActionsDb;
    std::string human_name = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
    textToPronounce = "Hey " + human_name + " . Please follow me to the destination";
  } else {
    textToPronounce = "Could you please follow me";
  }
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce, 1);
}

void aTellHumanObjectLocation(string params, bool* run) {
  string objectName;
  if (params == "GPSR") {
    database::GPSRActionsModel gpsrActionsDb;
    std_msgs::Int32 current_action_id_int32;
    bool is_value_available =
        SQLiteUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

    database::GPSRAction gpsrAction = gpsrActionsDb.getAction(current_action_id_int32.data);
    objectName = gpsrAction.object_item;
  } else
    objectName = params;
  ROS_INFO("The object name is: %s", objectName);
  std::string objName = robobreizh::convertCamelCaseToSpacedText(objectName);
  std::string textToPronounce = "The object " + objName + " is found successfully at the destination";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce, 1);
}

void aAskHumanTake(string params, bool* run) {
  string objNameNonProcessed;
  if (params == "GPSR") {
    GPSRActionsModel gpsrActionsDb;
    objNameNonProcessed = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item);
  } else
    objNameNonProcessed = params;
  std::string objName = robobreizh::convertCamelCaseToSpacedText(objNameNonProcessed);
  std::string textToPronounce = "Could you please help me taking the " + objName;
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce, 1);
}

void aAskActionConfirmation(string params, bool* run) {
  string textToPronounce = "Have you been able to help me? Please answer By Yes or No";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce, 1);
}

void aIntroduceAtoB(std::string params, bool* run) {
  // TODO: Replace with real names using database
  // Get Parameters
  int i_humanA = params.find("_");
  int i_humanB = params.find("_", i_humanA + 1);
  string humanA = params.substr(0, i_humanA);
  string humanB = params.substr(i_humanA + 1, i_humanB);

  ROS_INFO("aIntroduceAtoB - Introduce %s to %s", humanA.c_str(), humanB.c_str());

  robobreizh::database::PersonModel pm;
  if (humanA == "Guest") {
    robobreizh::database::Person guest = pm.getLastPerson();
    dialog::generic::robotSpeech("Here is our new guest.", 0);
    dialog::generic::presentPerson(guest);
  } else if (humanA == "Seated") {
    std::vector<robobreizh::database::Person> seatedPerson = pm.getPersons();
    dialog::generic::robotSpeech("Now. I will present you the person in the room.", 0);
    dialog::generic::presentPerson(seatedPerson);
  } else if (humanA == "Host") {
    int host_id = pm.getFirstPersonId();
    robobreizh::database::Person person = pm.getPerson(host_id);
    dialog::generic::robotSpeech("Now. I will present you the host.", 0);
    dialog::generic::presentPerson(person);
  } else if (humanA == "Guest1") {
    // get features of guest1 which is the 3rd last person in the db
    robobreizh::database::Person guest1 = pm.getPerson(pm.getLastPersonId() - 2);

    // this should first look for guest1
    // orientate himself directly in front of guest1
    // give description
    dialog::generic::robotSpeech("Here is our guest.", 0);
    dialog::generic::presentPerson(guest1);
  } else if (humanA == "Guest2") {
    robobreizh::database::Person guest2 = pm.getPerson(pm.getLastPersonId() - 1);
    dialog::generic::robotSpeech("Here is our guest.", 0);
    dialog::generic::presentPerson(guest2);
  } else {
    ROS_ERROR("Introduce A to B function entered an unknown condition");
  }

  // Gaze towards Human B (Gesture Generic Actions)

  // Small presentation sentence
  *run = true;
}

void aOfferSeatToHuman(string params, bool* run) {
  ROS_INFO("aOfferSeatToHuman - Offer seat to %s", params.c_str());

  // Gaze towards Human (Gesture Generic Actions)

  // Get Empty seat position from database
  robobreizh::database::ObjectModel om;
  database::Object object = om.getPositionByLabel("seat");

  geometry_msgs::PointStamped ps;
  ps.header.frame_id = "map";
  ps.point.x = (float)object.position.x;
  ps.point.y = (float)object.position.y;
  ps.point.z = (float)object.position.z;
  auto baselink_point = convert_point_stamped_to_frame(ps, "base_link");
  float distance = object.distance;
  std::cout<<"aOfferSeatToHuman" << endl;
  std::cout<< std::to_string(distance) << endl;
  // Point towards seat (Gesture Generic Action)
  robobreizh::gesture::generic::pointObjectPosition(baselink_point, distance);

  // Speech
  string sentence = params + ", Could you please sit there.";
  dialog::generic::robotSpeech(sentence, 1);
  RoboBreizhManagerUtils::pubVizBoxRobotText(sentence);

  // Gaze towards seat (joint attention)

  // Insert person in seated list
  robobreizh::database::PersonModel pm;
  auto last_person = pm.getLastPerson();
  int last_person_id = pm.getLastPersonId();
  last_person.posture = "seating";
  pm.updatePerson(last_person_id, last_person);

  RoboBreizhManagerUtils::setPNPConditionStatus("SeatOffered");
  *run = 1;
}

void aListenOrders(string params, bool* run) {
  // Empty GPSR Actions database
  database::GPSRActionsModel gpsrActionsDb;
  gpsrActionsDb.deleteAllActions();

  // Re-initialise action id counter
  g_order_index = 0;

  // Dialog - Speech-To-Text
  if (!dialog::generic::ListenSpeech()) {
    string pnpCondition = "NotUnderstood";
    *run = 1;
    RoboBreizhManagerUtils::setPNPConditionStatus(pnpCondition);
    return;
  }
  database::SpeechModel sm;
  std::string transcript = sm.getLastSpeech();
  std_msgs::String transcript_sentence;
  transcript_sentence.data = transcript;
  std_msgs::String corrected_sentence;
  string pnpCondition = "NotUnderstood";

  dialog::generic::robotSpeech("Please Correct And Confirm Your Order On The Screen", 1);
  // publish transcript_sentence to "rosservice /robobreizh/sentence_gpsr"
  if (!RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>("/robobreizh/sentence_gpsr", transcript_sentence)) {
    ROS_ERROR("Sending message to \"/robobreizh/sentence_gpsr\" failed");
  }
  if (RoboBreizhManagerUtils::waitForMessageFromTopic<std_msgs::String>("/robobreizh/sentence_gpsr_corrected",
                                                                        corrected_sentence)) {
    // retrieve the corrected value within the transcript variable
    ROS_INFO("The corrected transcript get from the client is: %s", corrected_sentence.data.c_str());

    int numberOfActions = 0;
    bool possible = true;

    std::vector<std::string> intent = dialog::generic::getIntent(corrected_sentence.data);
    if (!intent.empty()) {
      bool isTranscriptValid = generic::validateTranscriptActions(intent);

      if (!corrected_sentence.data.empty() && isTranscriptValid) {
        RoboBreizhManagerUtils::pubVizBoxOperatorText(corrected_sentence.data);

        // Add GPSR orders to database
        for (int i = 0; i < intent.size(); i++) {
          bool flag = true;
          database::GPSRAction gpsrAction = generic::getActionFromString(intent.at(i));
          if (gpsrAction.intent != "DEBUG_EMPTY") {
            numberOfActions++;
            gpsrActionsDb.insertAction(i + 1, gpsrAction);
          }
        }

        // Retrieve current position of the robot
        // add position to database with the location name = "me"
        geometry_msgs::PoseWithCovariance pose = navigation::generic::getCurrentPosition();
        database::LocationModel lm;

        database::Location me_location;
        me_location.angle = 0.0;
        me_location.pose = pose.pose;
        me_location.name = "me";
        me_location.frame = "map";
        me_location.room = { "" };

        if (lm.getLocationFromName("me").name.empty()) {
          lm.insertLocation(me_location);
        } else {
          lm.updateLocation(me_location);
        }

        // Modify value of total number of actions
        g_nb_action = numberOfActions;

        // Modify PNP Output status
        if (possible)
          pnpCondition = "Understood";
        else
          pnpCondition = "UnderstoodImpossible";
      } else {
        // Reinitialize number of actions
        g_nb_action = 0;
        pnpCondition = "NotUnderstood";
      }
    } else {
      pnpCondition = "NotUnderstood";
      ROS_ERROR("Failed to generate intents");
    }
  } else {
    pnpCondition = "NotUnderstood";
    ROS_ERROR("Failed to get the corrected_sentence from publisher");
  }

  // Dialog - Interpretation/extraction
  ROS_INFO("Hello aListenOrders, pnpCondition = %s", pnpCondition.c_str());

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpCondition);

  *run = 1;
}

void aListenConfirmation(string params, bool* run) {
  string pnpStatus = "NotUnderstood";

  // Dialog - Speech-To-Text
  const string SPEECH_SERVICE = "Confirmation";

  bool correct = true;
  std::string itemName = startSpecifiedListenSpeechService(SPEECH_SERVICE);

  if (itemName.empty()) {
    ROS_INFO("aListen - Item to listen not known");
    correct = false;
  }

  // Update user information in database if correct == true
  if (correct) {
    if (itemName == "yes") {
      if (params == "GPSR") {
        GPSRActionsModel gpsrActionsDb;
        std::string human_name = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
        database::PersonModel pm;
        database::Person person = pm.getLastPerson();  // pm.getPersonByName(human_name);
        person.name = human_name;
        pm.updatePerson(person.id, person);
      }
      pnpStatus = "UnderstoodYes";
    } else if (itemName == "no") {
      pnpStatus = "NotUnderstood";
    } else {
      pnpStatus = "NotUnderstood";
    }
  }

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
  *run = 1;
}  // namespace plan

std::string startSpecifiedListenSpeechService(std::string param) {
  std::array<std::string, 5> aItem = { "Name", "Drink", "Start", "Confirmation", "Arenanames" };
  std::string sentence = "";
  std::string itemName = "";
  for (const auto& item : aItem) {
    if (param == item) {
      // Dialog - Speech-To-Text
      if (!dialog::generic::ListenSpeech()) {
        return itemName;
      }
      database::SpeechModel sm;
      std::string transcript = sm.getLastSpeech();
      std::string itemName = dialog::generic::transcriptContains(item, transcript);
      RoboBreizhManagerUtils::pubVizBoxOperatorText(transcript.c_str());
      RoboBreizhManagerUtils::pubVizBoxOperatorText(item + " : " + itemName.c_str());
      ROS_INFO("aListen - Item listened : %s", itemName.c_str());
      return itemName;
    }
  }
  return itemName;
}

void aListen(std::string params, bool* run) {
#ifdef LEGACY
  const string PARAM_NAME_WHEREIS_FURNITURE = "param_whereisthis_furniture";
#endif
  bool correct = true;
  bool defaultValue = false;
  bool sqliteRet;
  std::string itemName = startSpecifiedListenSpeechService(params);

  if (itemName.empty()) {
    // If the number of failed recognitions reach the limit, choose default value and go on
    if (g_failure_counter >= 2) {
      if (params == "Name")
        itemName = g_default_name;
      else if (params == "Drink")
        itemName = g_default_drink;

      ROS_WARN("%d failed speech recognitions in a row. Set default -> %s = %s ", g_failure_limit, params.c_str(),
               itemName.c_str());
      correct = true;
      defaultValue = true;
    } else {
      ROS_INFO("aListen - %s to listen unknown (trials %d/2)", params.c_str(), g_failure_counter);
      g_failure_counter++;
      correct = false;
    }
  }
  // Update user information in database if correct == true
  if (correct) {
    // Update database here
    robobreizh::database::PersonModel pm;
    int last_person_id = pm.getLastPersonId();
    auto last_person = pm.getLastPerson();
    if (params == "Name") {
      last_person.name = itemName;
      pm.updatePerson(last_person_id, last_person);
      dialog::generic::robotSpeech("Hello, " + itemName + ".", 0);
    } else if (params == "Drink") {
      last_person.favorite_drink = itemName;
      pm.updatePerson(last_person_id, last_person);
    }
    g_failure_counter = 0;
  }

  string PnpStatus;
  if (correct) {
    PnpStatus = "Understood";
  } else {
    PnpStatus = "NotUnderstood";
  }

  if (defaultValue) {
    PnpStatus = "NotUnderstoodDefault";
  }
  RoboBreizhManagerUtils::setPNPConditionStatus(PnpStatus);
  *run = 1;
}

void aDescribeHuman(string params, bool* run) {
  string humanName = params;
  string PnpStatus;

#ifdef LEGACY
  if (humanName == "AllGuests") {
    robobreizh::database::PersonModel pm;
    robobreizh::database::ObjectModel om;
    auto personList = pm.getPersons();
    auto objectList = om.getObjects();
    ROS_INFO("aDescribeHuman - Describe Humans from Recognised list - FindMyMates task");
    if (!dialog::generic::presentFMMGuests(personList, objectList)) {
      PnpStatus = "NotTold";
    } else {
      RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    }
    PnpStatus = "Told";
    RoboBreizhManagerUtils::setPNPConditionStatus(PnpStatus);
  }
#endif
}

void aAskHumanNameConfirmation(string params, bool* run) {
  string humanName;

  if (params == "GPSR") {
    GPSRActionsModel gpsrActionsDb;
    humanName = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
  } else
    humanName = params;

  string textToPronounce = "Excuse me, are you " + humanName;
  *run = dialog::generic::robotSpeech(textToPronounce, 1);
}

void aTellHumanDestinationArrived(string params, bool* run) {
  // Get Parameters
  int i_human = params.find("_");
  int i_destination = params.find("_", i_human + 1);
  string humanName = params.substr(0, i_human);
  string destinationName = params.substr(i_human + 1, i_destination);

  if (humanName == "GPSR") {
    GPSRActionsModel gpsrActionsDbHuman;
    humanName = gpsrActionsDbHuman.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
  }

  if (destinationName == "GPSR") {
    GPSRActionsModel gpsrActionsDbDestination;
    destinationName = gpsrActionsDbDestination.getSpecificItemFromCurrentAction(GPSRActionItemName::destination);
  }

  string textToPronounce = humanName + ", We've arrived in the " + destinationName;
  *run = dialog::generic::robotSpeech(textToPronounce, 1);
}

void aAskOperatorHelpOrder(string params, bool* run) {
  // For restaurant task
  // Fetch Order from GPSR Database (database re-used for restaurant)
  string order = " ";

  // Ask for help
  string textToPronouce = "Excuse me, Can you please help me and put on the tray the following order " + order;
  *run = dialog::generic::robotSpeech(textToPronouce, 1);
}

void aGreet(string params, bool* run) {
  if (params == "GPSR") {
    GPSRActionsModel gpsrActionsDbHuman;
    std::string human_name = gpsrActionsDbHuman.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
    std::string textToPronounce;
    switch (rand() % 5) {
      case 0:
        textToPronounce = "Hello there " + human_name;
        break;
      case 1:
        textToPronounce = "Greetings " + human_name;
        break;
      case 2:
        textToPronounce = "Salutations " + human_name + ". Its a pleasure to make your acquaintance.";
        break;
      case 3:
        textToPronounce = "Good day " + human_name + ". Trust you are doing well today.";
        break;
      case 4:
        textToPronounce = "Hello " + human_name + ". Good to see you.";
        break;
      default:
        ROS_WARN("[aGreet] Random sentence generator didn't work for greeting");
        textToPronounce = "Hello there" + human_name;
        break;
    }
    dialog::generic::robotSpeech(textToPronounce, 1);
  }
  *run = 1;
}

#ifdef LEGACY
void aDialogChitChat(string params, bool* run) {
  string textToPronounce;
  // ChitChat inside
  textToPronounce = "Great -- another time for me to shine and make a friend.";
  dialog::generic::robotSpeech(textToPronounce, 0);
  textToPronounce = "Also, I couldn't help but see I never got any help navigating.";
  dialog::generic::robotSpeech(textToPronounce, 0);
  textToPronounce =
      "Maybe you're thinking, oh, Pepper's such a strong and noble paragon of skill, he can handle it by itself.";
  dialog::generic::robotSpeech(textToPronounce, 0);
  textToPronounce = "Which, most of the time, you would be totally right about.";
  dialog::generic::robotSpeech(textToPronounce, 0);
  textToPronounce = "Eventually we'll end up reaching that cab.";
  dialog::generic::robotSpeech(textToPronounce, 0);
}
#endif
}  // namespace plan
}  // namespace dialog
}  // namespace robobreizh
