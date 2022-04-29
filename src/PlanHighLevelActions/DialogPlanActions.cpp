#include <std_msgs/String.h>

#include "PlanHighLevelActions/DialogPlanActions.hpp"
#include "GenericActions/DialogGenericActions.hpp"
#include <boost/format.hpp>
#include <pnp_ros/names.h>

#include <ros/ros.h>

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
    // Dialog - Text-To-Speech
    *run = dialog::generic::robotSpeech("Ask Human to do " + params);
}

void aListenOrders(string params, bool* run)
{
    ros::NodeHandle handle;
    ros::Publisher pnp_condition_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);
    std_msgs::String cond;

    // Dialog - Speech-To-Text
    string transcript;
    transcript = dialog::generic::ListenSpeech();

    // Dialog - Interpretation/extraction

    cond.data = "Understood";
    pnp_condition_pub.publish(cond);
    *run = 1;
}



} // namespace generic
} // namespace plan
}// namespace robobreizh
