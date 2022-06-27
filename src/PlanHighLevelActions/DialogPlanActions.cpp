#include <std_msgs/String.h>
#include <boost/format.hpp>
#include <ros/ros.h>

#include <pnp_ros/names.h>

#include "PlanHighLevelActions/DialogPlanActions.hpp"
#include "GenericActions/DialogGenericActions.hpp"
#include "DatabaseModel/DialogModel.hpp"
#include "ManagerUtils.hpp"

using namespace std;

namespace robobreizh
{
namespace dialog
{
namespace plan
{

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

std::string convertCamelCaseToSpacedText(std::string params){
    std::string action;
    for (std::string::iterator it = params.begin(); it != params.end(); ++it)
    {
        if (std::isupper(*it))
        {
            action+= " "; 
        }
        action += *it;
    }
    return action;
}

void aAskHuman(string params, bool* run)
{
    // Dialog - Text-To-Speech
    std::string action = convertCamelCaseToSpacedText(params);
    std::string textToPronounce = "Can you please indicate your " + action;
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanToFollowToLocation(string params, bool* run)
{
    // might need to split the string or something
    std::string action = convertCamelCaseToSpacedText(params);
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
    std::string objName= convertCamelCaseToSpacedText(params);
    std::string textToPronounce = "The object named " + objName + " is there";
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskHumanTake(string params, bool* run)
{
    std::string objName= convertCamelCaseToSpacedText(params);
    std::string textToPronounce = "Can you please help me taking the " + objName;
    RoboBreizhManagerUtils::pubVizBoxRobotText(textToPronounce);
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aAskActionConfirmation(string params, bool* run)
{
    std::string textToPronounce = "Have you been able to help me?";
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
        dialog::generic::presentPerson(guest);
    } else if (humanA == "Seated"){
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
    // wait to avoid recording his voice
    ros::Duration(1).sleep(); 
    std::string sentence;

    // Dialog - Speech-To-Text
    std::vector<string> transcript;
    transcript = dialog::generic::ListenSpeech(&sentence);

    string pnpCondition;

    if (!transcript.empty()){
        pnpCondition = "Understood";
        RoboBreizhManagerUtils::pubVizBoxOperatorText(sentence);
        RoboBreizhManagerUtils::pubVizBoxChallengeStep(1);
    }
    else{
        pnpCondition = "NotUnderstood";
    }

   // Dialog - Interpretation/extraction
    RoboBreizhManagerUtils::setPNPConditionStatus(pnpCondition);
    
    *run = true;
}

void aListenConfirmation(string params, bool* run)
{
    // TODO Recovery: Add PNPStatus for UnderstoodNo and NotUnderstood
    
    // Dialog - Speech-To-Text

    RoboBreizhManagerUtils::setPNPConditionStatus("UnderstoodYes");
}

std::string startSpecifiedListenSpeechService(std::string param){
    std::array<std::string,2> aItem = {"Name","Drink"};
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

    // Find My Mates task
    if (humanName == "Guests")
    {
        ROS_INFO("aDescribeHuman - Describe Humans from Recognised list - FindMyMates task");
    }
}

} // namespace generic
} // namespace plan
}// namespace robobreizh
