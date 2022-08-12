#ifndef _PNP_ROBOBREIZH_DIALOG_PLAN_ACTIONS_
#define _PNP_ROBOBREIZH_DIALOG_PLAN_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh
{
namespace dialog
{
namespace plan
{
std::string convertCamelCaseToSpacedText(std::string params);
void aAskHuman(std::string params, bool* run);
void aListenOrders(std::string params, bool* run);
void aListenConfirmation(std::string params, bool* run);
std::string startSpecifiedListenSpeechService(std::string param);
void aListen(std::string params, bool* run);
void aAskHumanToFollow(std::string params, bool* run);
void aAskHumanToFollowToLocation(std::string params, bool* run);
void aTellHumanObjectLocation(std::string params, bool* run);
void aAskHumanTake(std::string params, bool* run);
void aAskActionConfirmation(std::string params, bool* run);
void aIntroduceAtoB(std::string params, bool* run);
void aOfferSeatToHuman(std::string params, bool* run);
void aDescribeHuman(std::string params, bool* run);
void aAskHumanToStartTask(std::string params, bool* run);
void aAskHumanRepeat(std::string params, bool* run);
void aSay(std::string params, bool* run);
void aAskHumanNameConfirmation(std::string params, bool* run);
void aTellHumanDestinationArrived(std::string params, bool* run);
void aAskOperatorHelpOrder(std::string params, bool* run);
void aDialogAskHumanTakeLastObject(std::string params, bool* run);
void aDialogChitChat(std::string params, bool* run);
void aDialogAskHumanPlaceLastObjectOnTablet(std::string params, bool* run);
}  // namespace plan
}  // namespace dialog
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_DIALOG_PLAN_ACTIONS_
