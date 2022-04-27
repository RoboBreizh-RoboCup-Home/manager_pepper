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
    *run = generic::robotSpeech("Hello, I'll be your butler today.");
}

void aAskHandOverObject(std::string params, bool* run)
{
    string object = params;
    string textToPronounce = "I need your help, can you please hand me over " + object;
    *run = generic::robotSpeech(textToPronounce);
}



} // namespace generic
} // namespace plan
}// namespace robobreizh
