#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <boost/format.hpp>
#include <ros/ros.h>

#include <pnp_ros/names.h>

#include "plan_high_level_actions/dialog_plan_actions.hpp"
#include "generic_actions/dialog_generic_actions.hpp"
#include "database_model/person_model.hpp"
#include "database_model/object_model.hpp"
#include "database_model/speech_model.hpp"
#include "database_model/gpsr_actions_model.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"

using namespace std;

using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

namespace robobreizh
{
namespace dialog
{
namespace plan
{

/**
 * @brief aSay takes a string as an argument from the petri net and say it using naoqi api.
 *
 * @param params string to say in camel case.
 * @param run boolean value allowing to change the state of the petri net.
 */
void aSay(string params, bool* run)
{
  std::string text = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(params);
  RoboBreizhManagerUtils::pubVizBoxRobotText(text);
  *run = dialog::generic::robotSpeech(text);
}

/**
 * @brief ask human to place object last object.
 *
 * @param params
 * @param run
 */
void aDialogAskHumanPlaceLastObjectOnTablet(string params, bool* run)
{
  robobreizh::database::ObjectModel om;
  robobreizh::database::Object obj = om.getLastObject();
  std::string text = "Could you please put the " + obj.label + " on the tablet";
  robobreizh::dialog::generic::robotSpeech(text);
  ROS_INFO(text.c_str());
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

void aDialogAskHumanTakeLastObject(string params, bool* run)
{
  robobreizh::database::ObjectModel om;
  robobreizh::database::Object obj = om.getLastObject();
  std::cout << obj.label << std::endl;
  std::string text = "Could you please take the " + obj.label + " with you.";
  robobreizh::dialog::generic::robotSpeech(text);
  ROS_INFO(text.c_str());
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = 1;
}

void aAskHuman(string params, bool* run)
{
  // Dialog - Text-To-Speech
  std::string action = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(params);
  std::string textToPronounce =
      "Could you please indicate your " + action + ". Would you kindly speak as loud as possible";

  // Specific cases
  if (params == "waveHandFarewell")
    textToPronounce = "Could you please wave your hands if you want to leave";

  if (params == "waveHand")
    textToPronounce = "I can't see you, could you please wave your hand";

  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanRepeat(string params, bool* run)
{
  std::string textToPronounce = "Sorry, I didn't understand. Could you please repeat";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanToStartTask(string params, bool* run)
{
  std::string textToPronounce = "To start the task please say : 'start the task'";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanToFollowToLocation(string params, bool* run)
{
  // might need to split the string or something
  std::string action = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(params);
  std::string textToPronounce = "Could you please follow me to the " + action;
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanToFollow(string params, bool* run)
{
  std::string textToPronounce = "Could you please follow me";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aTellHumanObjectLocation(string params, bool* run)
{
  string objectName;
  if (params == "GPSR")
  {
    database::GPSRActionsModel gpsrActionsDb;
    std_msgs::Int32 current_action_id_int32;
    bool is_value_available =
        SQLiteUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

    database::GPSRAction gpsrAction = gpsrActionsDb.getAction(current_action_id_int32.data);
    objectName = gpsrAction.object_item;
  }
  else
    objectName = params;

  std::string objName = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(objectName);
  std::string textToPronounce = "The object named " + objName + " is there";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanTake(string params, bool* run)
{
  string objNameNonProcessed;
  if (params == "GPSR")
  {
    GPSRActionsModel gpsrActionsDb;
    objNameNonProcessed = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::object_item);
  }
  else
    objNameNonProcessed = params;
  std::string objName = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(objNameNonProcessed);
  std::string textToPronounce = "Could you please help me taking the " + objName;
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskActionConfirmation(string params, bool* run)
{
  string textToPronounce = "Have you been able to help me? Please answer By Yes or No";
  RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aIntroduceAtoB(std::string params, bool* run)
{
  // TODO: Replace with real names using database
  // Get Parameters
  int i_humanA = params.find("_");
  int i_humanB = params.find("_", i_humanA + 1);
  string humanA = params.substr(0, i_humanA);
  string humanB = params.substr(i_humanA + 1, i_humanB);

  ROS_INFO("aIntroduceAtoB - Introduce %s to %s", humanA.c_str(), humanB.c_str());

  robobreizh::database::PersonModel pm;
  if (humanA == "Guest")
  {
    robobreizh::database::Person guest = pm.getLastPerson();
    dialog::generic::robotSpeech("Here is our new guest.");
    dialog::generic::presentPerson(guest);
  }
  else if (humanA == "Seated")
  {
    std::vector<robobreizh::database::Person> seatedPerson = pm.getPersons();
    dialog::generic::robotSpeech("Now. I will present you the person in the room.");
    dialog::generic::presentPerson(seatedPerson);
  }
  else if (humanA == "Host")
  {
    int host_id = pm.getFirstPersonId();
    robobreizh::database::Person person = pm.getPerson(host_id);
    dialog::generic::robotSpeech("Now. I will present you the host.");
    dialog::generic::presentPerson(person);
  }

  // Gaze towards Human B (Gesture Generic Actions)

  // Small presentation sentence
  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  *run = true;
}

void aOfferSeatToHuman(string params, bool* run)
{
  ROS_INFO("aOfferSeatToHuman - Offer seat to %s", params.c_str());

  // Gaze towards Human (Gesture Generic Actions)

  // Get Empty seat position from database

  // Point towards seat (Gesture Generic Action)
  system("rosservice call /robobreizh/manipulation/point_in_front");

  // Speech
  string sentence = params + ", Could you please sit there.";
  dialog::generic::robotSpeech(sentence);
  RoboBreizhManagerUtils::pubVizBoxRobotText(sentence);

  // Gaze towards seat (joint attention)

  // Insert person in seated list
  robobreizh::database::PersonModel pm;
  auto last_person = pm.getLastPerson();
  int last_person_id = pm.getLastPersonId();
  last_person.posture = "seating";
  pm.updatePerson(last_person_id, last_person);

  RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  RoboBreizhManagerUtils::setPNPConditionStatus("SeatOffered");
  *run = 1;
}

void aListenOrders(string params, bool* run)
{
  // Empty GPSR Actions database
  database::GPSRActionsModel gpsrActionsDb;
  gpsrActionsDb.deleteAllActions();

  // Re-initialise action id counter
  std_msgs::Int32 current_action_id_int32;
  current_action_id_int32.data = 0;
  bool ret = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

  // Dialog - Speech-To-Text
  if (!dialog::generic::ListenSpeech())
  {
    string pnpCondition = "NotUnderstood";
    *run = 1;
    RoboBreizhManagerUtils::setPNPConditionStatus(pnpCondition);
    return;
  }
  database::SpeechModel sm;
  std::string transcript = sm.getLastSpeech();

  string pnpCondition = "NotUnderstood";
  int numberOfActions = 0;
  bool possible = true;

  std::vector<std::string> intent = dialog::generic::getIntent(transcript);
  bool isTranscriptValid = generic::validateTranscriptActions(intent);

  if (!transcript.empty() && isTranscriptValid)
  {
    RoboBreizhManagerUtils::pubVizBoxOperatorText(transcript);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);

    // Add GPSR orders to database
    for (int i = 0; i < intent.size(); i++)
    {
      bool flag = true;
      database::GPSRAction gpsrAction = generic::getActionFromString(intent.at(i));
      if (gpsrAction.intent != "DEBUG_EMPTY")
      {
        numberOfActions++;
        if (gpsrAction.intent == "take")
        {
          if (gpsrAction.object_item.empty() && gpsrAction.person.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "go")
        {
          if (gpsrAction.destination.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "follow")
        {
          if (gpsrAction.person.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "to find something")
        {
          if (gpsrAction.object_item.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "to find someone")
        {
          if (gpsrAction.person.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "say")
        {
          possible = false;
          if (gpsrAction.what.empty())
            flag = false;
        }

        gpsrActionsDb.insertAction(i + 1, gpsrAction);
      }
    }

    // Modify value of total number of actions
    std_msgs::Int32 number_actions;
    number_actions.data = numberOfActions;
    ret = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("param_gpsr_nb_actions", number_actions);

    // Modify PNP Output status
    if (possible)
      pnpCondition = "Understood";
    else
      pnpCondition = "UnderstoodImpossible";
  }
  else
  {
    // Reinitialise number of actions
    std_msgs::Int32 number_actions;
    number_actions.data = 0;
    bool ret = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("param_gpsr_nb_actions", number_actions);

    // Modify PNP Output status
    pnpCondition = "NotUnderstood";
  }

  // Dialog - Interpretation/extraction
  ROS_INFO("Hello aListenOrders, pnpCondition = %s", pnpCondition.c_str());

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpCondition);

  *run = 1;
}

void aListenConfirmation(string params, bool* run)
{
  string pnpStatus = "NotUnderstood";

  // Dialog - Speech-To-Text
  const string SPEECH_SERVICE = "Confirmation";

  bool correct = true;
  std::string itemName = startSpecifiedListenSpeechService(SPEECH_SERVICE);

  if (itemName.empty())
  {
    ROS_INFO("aListen - Item to listen not known");
    correct = false;
  }

  // Update user information in database if correct == true
  if (correct)
  {
    if (itemName == "yes")
      pnpStatus = "UnderstoodYes";
    else if (itemName == "no")
      pnpStatus = "UnderstoodNo";
    else
      pnpStatus = "NotUnderstood";
  }

  RoboBreizhManagerUtils::setPNPConditionStatus(pnpStatus);
  *run = 1;
}

std::string startSpecifiedListenSpeechService(std::string param)
{
  std::array<string, 5> aItem = { "Name", "Drink", "Start", "Confirmation", "Arenanames" };
  std::string sentence = "";
  std::string itemName = "";
  for (const auto& item : aItem)
  {
    if (param == item)
    {
      // Dialog - Speech-To-Text
      if (!dialog::generic::ListenSpeech())
      {
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

void aListen(string params, bool* run)
{
  const string PARAM_NAME_SPEECH_UNSUCCESSFUL_TRIES = "param_number_of_unsuccessful_tries";
  const string PARAM_NAME_WHEREIS_FURNITURE = "param_whereisthis_furniture";
  bool correct = true;
  bool defaultValue = false;
  bool sqliteRet;
  std::string itemName = startSpecifiedListenSpeechService(params);

  std_msgs::Int32 numberFailedSpeechTries;
  sqliteRet =
      SQLiteUtils::getParameterValue<std_msgs::Int32>(PARAM_NAME_SPEECH_UNSUCCESSFUL_TRIES, numberFailedSpeechTries);

  if (itemName.empty())
  {
    // If more than three failed recognitions in a row, choose default value and go on
    if (numberFailedSpeechTries.data >= 1)
    {
      ROS_INFO("Three failed speech recogntions in a row, we use default value instead to continue the task");
      if (params == "Name")
        itemName = "Parker";
      else if (params == "Drink")
        itemName = "Coffee";

      correct = true;
      defaultValue = true;
    }

    else
    {
      ROS_INFO("aListen - Item to listen not known");
      numberFailedSpeechTries.data++;
      sqliteRet = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>(PARAM_NAME_SPEECH_UNSUCCESSFUL_TRIES,
                                                                         numberFailedSpeechTries);
      correct = false;
    }
  }
  // Update user information in database if correct == true
  if (correct)
  {
    // Update database here
    robobreizh::database::PersonModel pm;
    int last_person_id = pm.getLastPersonId();
    auto last_person = pm.getLastPerson();
    if (params == "Name")
    {
      last_person.name = itemName;
      pm.updatePerson(last_person_id, last_person);
      dialog::generic::robotSpeech("Hello, " + itemName + ".");
    }
    else if (params == "Drink")
    {
      last_person.favorite_drink = itemName;
      pm.updatePerson(last_person_id, last_person);
    }
    else if (params == "Arenanames")
    {
      std_msgs::String furnitureData;
      furnitureData.data = itemName;
      sqliteRet = SQLiteUtils::modifyParameterParameter<std_msgs::String>(PARAM_NAME_WHEREIS_FURNITURE, furnitureData);
    }
    numberFailedSpeechTries.data = 0;
    sqliteRet = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>(PARAM_NAME_SPEECH_UNSUCCESSFUL_TRIES,
                                                                       numberFailedSpeechTries);
  }

  string PnpStatus;
  if (correct)
  {
    PnpStatus = "Understood";
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
  }
  else
  {
    PnpStatus = "NotUnderstood";
  }

  if (defaultValue)
  {
    PnpStatus = "NotUnderstoodDefault";
  }
  RoboBreizhManagerUtils::setPNPConditionStatus(PnpStatus);
  *run = 1;
}

void aDescribeHuman(string params, bool* run)
{
  string humanName = params;
  string PnpStatus;

  // Find My Mates
  if (humanName == "AllGuests")
  {
    robobreizh::database::PersonModel pm;
    robobreizh::database::ObjectModel om;
    auto personList = pm.getPersons();
    auto objectList = om.getObjects();
    ROS_INFO("aDescribeHuman - Describe Humans from Recognised list - FindMyMates task");
    if (!dialog::generic::presentFMMGuests(personList, objectList))
    {
      PnpStatus = "NotTold";
    }
    else
    {
      RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    }
    PnpStatus = "Told";
    RoboBreizhManagerUtils::setPNPConditionStatus(PnpStatus);
  }
}

void aAskHumanNameConfirmation(string params, bool* run)
{
  string humanName;

  if (params == "GPSR")
  {
    GPSRActionsModel gpsrActionsDb;
    humanName = gpsrActionsDb.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
  }
  else
    humanName = params;

  string textToPronounce = "Excuse me, are you " + humanName;
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aTellHumanDestinationArrived(string params, bool* run)
{
  // Get Parameters
  int i_human = params.find("_");
  int i_destination = params.find("_", i_human + 1);
  string humanName = params.substr(0, i_human);
  string destinationName = params.substr(i_human + 1, i_destination);

  if (humanName == "GPSR")
  {
    GPSRActionsModel gpsrActionsDbHuman;
    humanName = gpsrActionsDbHuman.getSpecificItemFromCurrentAction(GPSRActionItemName::person);
  }

  if (destinationName == "GPSR")
  {
    GPSRActionsModel gpsrActionsDbDestination;
    destinationName = gpsrActionsDbDestination.getSpecificItemFromCurrentAction(GPSRActionItemName::destination);
  }

  string textToPronounce = humanName + ", We've arrived in the " + destinationName;
  *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskOperatorHelpOrder(string params, bool* run)
{
  // For restaurant task
  // Fetch Order from GPSR Database (database re-used for restaurant)
  string order = " ";

  // Ask for help
  string textToPronouce = "Excuse me, Can you please help me and put on the tray the following order " + order;
  *run = dialog::generic::robotSpeech(textToPronouce);
}

void aDialogChitChat(string params, bool* run)
{
  string textToPronounce;
  // ChitChat inside
  textToPronounce = "Great -- another time for me to shine and make a friend.";
  dialog::generic::robotSpeech(textToPronounce);
  textToPronounce = "Also, I couldn't help but see I never got any help navigating.";
  dialog::generic::robotSpeech(textToPronounce);
  textToPronounce =
      "Maybe you're thinking, oh, Pepper's such a strong and noble paragon of skill, he can handle it by itself.";
  dialog::generic::robotSpeech(textToPronounce);
  textToPronounce = "Which, most of the time, you would be totally right about.";
  dialog::generic::robotSpeech(textToPronounce);
  textToPronounce = "Eventually we'll end up reaching that cab.";
  dialog::generic::robotSpeech(textToPronounce);
}
}  // namespace plan
}  // namespace dialog
}  // namespace robobreizh
