#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <boost/format.hpp>
#include <ros/ros.h>

#include <pnp_ros/names.h>

#include "PlanHighLevelActions/DialogPlanActions.hpp"
#include "GenericActions/DialogGenericActions.hpp"
#include "DatabaseModel/DialogModel.hpp"
#include "DatabaseModel/VisionModel.hpp"
#include "ManagerUtils.hpp"
#include "SQLiteUtils.hpp"
#include "DatabaseModel/GPSRActionsModel.hpp"

using namespace std;

using GPSRActionsModel = robobreizh::database::GPSRActionsModel;
using GPSRActionItemName = robobreizh::database::GPSRActionItemName;

namespace robobreizh
{
namespace dialog
{
namespace plan
{

void aSay(string params,bool *run){
    std::string text = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(params);
    RoboBreizhManagerUtils::pubVizBoxRobotText(text);
    *run = dialog::generic::robotSpeech(text);
}

void aGreetHuman(string params, bool* run)
{
    // Dialog - Text-To-Speech
    string textToPronounce = "Hello, I'll be your butler today.";
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHandOverObject(string params, bool* run)
{
    // Get parameter(s)
    string object = params;

    // Dialog - Text-To-Speech
    string textToPronounce = "I need your help. Could you please hand me over " + object;
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aTellReadyToGo(string params, bool* run)
{
    // Dialog - Text-To-Speech
    std::string textToPronounce = "Thank you for your patience, we're ready to go";
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aTellGoodbye(string params, bool* run)
{
    // Dialog - Text-To-Speech
    std::string textToPronounce = "Thank you, have a nice day!";
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aDialogAskHumanPlaceLastObjectOnTablet(string params, bool * run){
    robobreizh::database::VisionModel vm;
    Object obj = vm.getLastObject();
    robobreizh::dialog::generic::robotSpeech("Can you please put the " + obj.label + " on the tablet");

    ROS_INFO("Ask to put the object on the tablet done");
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = 1;
}


void aAskHuman(string params, bool* run)
{
    // Dialog - Text-To-Speech
    std::string action = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(params);
    std::string textToPronounce = "Can you please indicate your " + action;
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanRepeat(string params,bool* run){
    std::string textToPronounce = "Sorry, I didn't understand. Could you please repeat"; 
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanToStartTask(string params, bool* run){
    std::string textToPronounce = "To start the task please say : 'start the task'"; 
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanToFollowToLocation(string params, bool* run)
{
    // might need to split the string or something
    std::string action = RoboBreizhManagerUtils::convertCamelCaseToSpacedText(params);
    std::string textToPronounce = "Can you please follow me to the " + action;
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanToFollow(string params, bool* run)
{
    std::string textToPronounce = "Can you please follow me"; 
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
        bool is_value_available = SQLiteUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

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
    std::string textToPronounce = "Can you please help me taking the " + objName;
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskActionConfirmation(string params, bool* run)
{
    string textToPronounce = "Have you been able to help me? Please answer By Yes or No";
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aIntroduceAtoB(std::string params, bool* run)
{
    // TODO: Replace with real names using database
    // Get Parameters
    robobreizh::database::DialogModel dm;
    Person guest = dm.getLastPersonWithName();
    std::vector<Person> seatedPerson = dm.getSeatedPerson();
    int i_humanA=params.find("_");
    int i_humanB=params.find("_", i_humanA + 1);
    string humanA=params.substr(0, i_humanA);
    string humanB=params.substr(i_humanA + 1, i_humanB);

    ROS_INFO("aIntroduceAtoB - Introduce %s to %s", humanA.c_str(),humanB.c_str());

    if (humanA == "Guest"){
        dialog::generic::robotSpeech("Here is our new guest.");
        dialog::generic::presentPerson(guest);
    } else if (humanA == "Seated"){
        dialog::generic::robotSpeech("Now. I will present you the person in the room.");
        dialog::generic::presentPerson(seatedPerson);
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
    string sentence = params + ", You can sit there.";
    dialog::generic::robotSpeech(sentence);
    RoboBreizhManagerUtils::pubVizBoxRobotText(sentence);

    // Gaze towards seat (joint attention)

    // Insert person in seated list
    robobreizh::database::DialogModel dm;
    dm.insertSeatedPerson();

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

    // wait to avoid recording his voice
    ros::Duration(1).sleep(); 
    std::string sentence;

    // Dialog - Speech-To-Text
    std::vector<string> transcript;
    transcript = dialog::generic::ListenSpeech(&sentence);

    string pnpCondition = "NotUnderstood";
    int numberOfActions = 0;
    if (!transcript.empty()){
        RoboBreizhManagerUtils::pubVizBoxOperatorText(sentence);
        RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);

        // Add GPSR orders to database
        for (int i = 0; i < transcript.size(); i++)
        {
            database::GPSRAction gpsrAction = generic::getActionFromString(transcript.at(i));
            if (gpsrAction.intent != "DEBUG_EMPTY")
            {
                numberOfActions++;
                gpsrActionsDb.insertAction(i + 1, gpsrAction);
            }
                
        }

        // Modify value of total number of actions
        std_msgs::Int32 number_actions;
        number_actions.data = numberOfActions;
        ret = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("param_gpsr_nb_actions", number_actions);

        // Modify PNP Output status
        pnpCondition = "Understood";
    }
    else{
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
    std::vector<string> transcript;

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

std::string startSpecifiedListenSpeechService(std::string param){
    std::array<string, 4> aItem = {"Name","Drink","Start", "Confirmation"};
    std::string sentence;
    std::string itemName;
    for (const auto& item: aItem){
        if (param == item)
        {
            itemName = dialog::generic::ListenSpeech(param, &sentence);
            RoboBreizhManagerUtils::pubVizBoxOperatorText(sentence.c_str());
            RoboBreizhManagerUtils::pubVizBoxOperatorText(item + " : " + itemName.c_str());
            ROS_INFO("aListen - Item listened : %s",itemName.c_str());
            return itemName;
        }
    }
    return itemName;
}

void aListen(string params, bool* run)
{
    std::vector<string> transcript;

    bool correct = true;
    std::string itemName = startSpecifiedListenSpeechService(params);

    if (itemName.empty())    
    {
        ROS_INFO("aListen - Item to listen not known");
        correct = false;
    }
    
    // Update user information in database if correct == true
    if (correct)
    {
        // Update database here
        robobreizh::database::DialogModel dm;
        if (params == "Name")
        {
            dm.updatePersonName(itemName);
            dialog::generic::robotSpeech("Hello, "+itemName+".");

        } else if (params == "Drink"){
            dm.updatePersonFavoriteDrink(itemName);
        } 
    }

    string PnpStatus;
    if (correct){
        PnpStatus = "Understood";
        RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    }else{
        PnpStatus = "NotUnderstood";
    }
    RoboBreizhManagerUtils::setPNPConditionStatus(PnpStatus);
    *run = 1;
}

void aDescribeHuman(string params, bool* run)
{
    string humanName = params;
    string PnpStatus;

    // Find My Mates task
    if (humanName == "AllGuests")
    {
        robobreizh::database::VisionModel vm;
        auto personList = vm.getAllPerson();
        auto objectList = vm.getAllObject();
        ROS_INFO("aDescribeHuman - Describe Humans from Recognised list - FindMyMates task");
        if (!dialog::generic::presentFMMGuests(personList,objectList)){
            PnpStatus = "NotTold";
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
} // namespace generic
} // namespace plan
}// namespace robobreizh
