#include <std_msgs/String.h>

#include "PlanHighLevelActions/DialogPlanActions.hpp"
#include "GenericActions/DialogGenericActions.hpp"
#include <boost/format.hpp>

using namespace std;

namespace robobreizh
{
namespace dialog
{
namespace plan
{

void aGreetHuman(string params, bool* run)
{
    // Dialog - Speech-To-Text
    *run = dialog::generic::robotSpeech("Hello, I'll be your butler today.");
}

void aAskHandOverObject(string params, bool* run)
{
    // Get parameter(s)
    string object = params;

    // Dialog - Speech-To-Text
    string textToPronounce = "I need your help, can you please hand me over " + object;
    *run = dialog::generic::robotSpeech(textToPronounce);
}

void aTellReadyToGo(string params, bool* run)
{
    // Dialog - Speech-To-Text
    *run = dialog::generic::robotSpeech("Thank you for your patience, we're ready to go");
}
void aTellGoodbye(string params, bool* run)
{
    // Dialog - Speech-To-Text
    *run = dialog::generic::robotSpeech("Thank you, have a nice day!");
}

} // namespace generic
} // namespace plan
}// namespace robobreizh
