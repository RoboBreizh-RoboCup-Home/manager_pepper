#include <std_msgs/String.h>
#include <boost/format.hpp>
#include <ros/ros.h>

#include <pnp_ros/names.h>

#include "PlanHighLevelActions/DialogPlanActions.hpp"
#include "GenericActions/DialogGenericActions.hpp"
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
    string textToPronounce = "I need your help, can you please hand me over " + object;
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

void aAskHuman(string params, bool* run)
{
    string action = params;

    // Dialog - Text-To-Speech
    *run = dialog::generic::robotSpeech("Can you please " + action);
}

void aListenOrders(string params, bool* run)
{
    // Dialog - Speech-To-Text
    string transcript;
    transcript = dialog::generic::ListenSpeech();

    // Dialog - Interpretation/extraction
    RoboBreizhManagerUtils::setPNPConditionStatus("Understood");
}

void aListenConfirmation(string params, bool* run)
{
    // TODO Recovery: Add PNPStatus for UnderstoodNo and NotUnderstood
    
    // Dialog - Speech-To-Text

    RoboBreizhManagerUtils::setPNPConditionStatus("UnderstoodYes");
}

void aAskHumanToFollow(string params, bool* run)
{
    *run = dialog::generic::robotSpeech("Can you please follow me");
}

void aTellHumanObjectLocation(string params, bool* run)
{
    string objName = params;
    *run = dialog::generic::robotSpeech("The object named " + objName + " is there");
}

void aAskHumanTake(string params, bool* run)
{
    string objName = params;
    *run = dialog::generic::robotSpeech("Can you please help me to take the " + objName);
}

void aAskActionConfirmation(string params, bool* run)
{
    *run = dialog::generic::robotSpeech("Have you been able to help me?");
}
} // namespace generic
} // namespace plan
}// namespace robobreizh
