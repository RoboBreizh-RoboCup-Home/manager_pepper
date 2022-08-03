#ifndef _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include "DatabaseModel/VisionModel.hpp"
#include "DatabaseModel/GPSRActionsModel.hpp"

namespace robobreizh
{
namespace dialog
{
namespace generic
{
bool robotSpeech(std::string text);
std::vector<std::string> ListenSpeech(std::string* sentence);
std::string ListenSpeech(std::string param, std::string* sentence);
std::vector<std::string> wavToIntent(std::string*);
std::string wavToParsedParam(std::string param, std::string* sentence);
bool presentPerson(std::vector<Person> listPerson);
bool presentPerson(Person person);
std::string cleanString(std::string& str);
database::GPSRAction getActionFromString(std::string& str);
bool presentFMMGuests(std::vector<Person> listPerson, std::vector<Object> listObject);
bool validateTranscriptActions(std::vector<std::string>& transcript);

bool isValidObject(std::string objName);
bool isValidPlace(std::string placeName);
std::string whereIsThisBegin(std::string furniture, std::string startingLocation);
std::string goFromOfficeToKitchen();
std::string goFromKitchenToOffice();
std::string goFromLivingRoomToBedroom();
std::string goFromBedroomToLivingRoom();
std::string goFromLivingRoomToKitchen();
std::string goFromLivingRoomToOffice();
std::string goFromOfficeToLivingRoom();
std::string goFromOfficeToBedroom();
std::string goFromBedroomToKitchen();
std::string goFromBedroomToOffice();
std::string goFromKitchenToBedroom();
std::string goFromKitchenToLivingRoom();
void whereIsThisEnd(std::string furniture);
}  // namespace generic
}  // namespace dialog
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_
