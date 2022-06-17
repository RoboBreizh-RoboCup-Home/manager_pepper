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
    *run = dialog::generic::robotSpeech("Hello, I'll be your butler today.");
}

void aAskHandOverObject(string params, bool* run)
{
    // Get parameter(s)
    string object = params;

    // Dialog - Text-To-Speech
    string textToPronounce = "I need your help. Could you please hand me over " + object;
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aTellReadyToGo(string params, bool* run)
{
    // Dialog - Text-To-Speech
    *run = dialog::generic::robotSpeech("Thank you for your patience, we're ready to go");
}

void aTellGoodbye(string params, bool* run)
{
    // Dialog - Text-To-Speech
    *run = dialog::generic::robotSpeech("Thank you, have a nice day!");
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
    *run = dialog::generic::robotSpeech("Can you please indicate your " + action);
}

void aAskHumanToFollowToLocation(string params, bool* run)
{
    // might need to split the string or something
    std::string action = convertCamelCaseToSpacedText(params);
    *run = dialog::generic::robotSpeech("Can you please follow me to the " + action);
}

void aAskHumanToFollow(string params, bool* run)
{
    *run = dialog::generic::robotSpeech("Can you please follow me");
}

void aTellHumanObjectLocation(string params, bool* run)
{
    std::string objName= convertCamelCaseToSpacedText(params);
    *run = dialog::generic::robotSpeech("The object named " + objName + " is there");
}

void aAskHumanTake(string params, bool* run)
{
    std::string objName= convertCamelCaseToSpacedText(params);
    *run = dialog::generic::robotSpeech("Can you please help me taking the " + objName);
}

void aAskActionConfirmation(string params, bool* run)
{
    *run = dialog::generic::robotSpeech("Have you been able to help me?");
}

void aIntroduceAtoB(std::string params, bool* run)
{
    // TODO: Replace with real names using database
    // Get Parameters
    robobreizh::database::DialogModel dm;
    Person personA = dm.getLastPersonWithName();
    /* int i_humanA=params.find("_"); */
    /* int i_humanB=params.find("_", i_humanA + 1); */
    /* string humanA=params.substr(0, i_humanA); */
    /* string humanB=params.substr(i_humanA + 1, i_humanB); */

    ROS_INFO("aIntroduceAtoB - Introduce %s ", personA.name.c_str());
    
    // Gaze towards Human B (Gesture Generic Actions)

    // Small presentation sentence
    string sentence = " Here is " + personA.name + ". ";
    string pronoun;
    if (personA.gender.compare("male")){
        pronoun = "he";
        sentence += pronoun + " is a guy."; 
    } else {
        pronoun = "she";
        sentence += pronoun + " is a girl."; 
    }

    sentence += pronoun + " likes drinking " + personA.favorite_drink + ". ";
    sentence += pronoun + " is between " + personA.age+ " years old. ";
    sentence += pronoun + " wears " + personA.cloth_color+ " cloth. ";
    sentence += pronoun + " skin is " + personA.skin_color;

    *run = dialog::generic::robotSpeech(sentence);
}

void aOfferSeatToHuman(string params, bool* run)
{
    ROS_INFO("aOfferSeatToHuman - Offer seat to %s", params.c_str());

    // Gaze towards Human (Gesture Generic Actions)

    // Get Empty seat position from database

    // Point towards seat (Gesture Generic Action)

    // Speech
    string sentence = params + ", You can sit there.";
    dialog::generic::robotSpeech(sentence);

    // Gaze towards seat (joint attention)

    // Insert person in seated list
    robobreizh::database::DialogModel dm;
    dm.insertSeatedPerson();

    RoboBreizhManagerUtils::setPNPConditionStatus("SeatOffered");
    *run = 1;
}
void aListenOrders(string params, bool* run)
{
    // wait to avoid recording his voice
    ros::Duration(0.5).sleep(); 
    ROS_INFO("Inside AListenOrders");
    // Dialog - Speech-To-Text
    std::vector<string> transcript;
    transcript = dialog::generic::ListenSpeech();

    string pnpCondition;

    if (!transcript.empty())
        pnpCondition = "Understood";
    else
        pnpCondition = "NotUnderstood";


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
    std::string itemName;
    for (const auto& item: aItem){
        if (param == item)
        {
            itemName = dialog::generic::ListenSpeech(param);
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
        } else if (params == "Drink"){
            dm.updatePersonFavoriteDrink(itemName);
        }
    }

    string PnpStatus;
    if (correct)
        PnpStatus = "Understood";
    else
        PnpStatus = "NotUnderstood";
    RoboBreizhManagerUtils::setPNPConditionStatus(PnpStatus);
    *run = 1;
}

} // namespace generic
} // namespace plan
}// namespace robobreizh
