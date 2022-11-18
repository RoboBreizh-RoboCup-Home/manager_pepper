#ifndef _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include "database_model/person_model.hpp"
#include "database_model/object_model.hpp"
#include "database_model/gpsr_actions_model.hpp"

namespace robobreizh
{
namespace dialog
{
namespace generic
{
bool robotSpeech(std::string text);
std::vector<std::string> ListenSpeech(std::string* sentence);
bool ListenSpeech();
std::vector<std::string> wavToIntent(std::string*);
std::string transcriptContains(std::string category, std::string transcript);
bool presentPerson(std::vector<robobreizh::database::Person> listPerson);
bool presentPerson(robobreizh::database::Person person);
std::string cleanString(std::string& str);
database::GPSRAction getActionFromString(std::string& str);
bool presentFMMGuests(std::vector<robobreizh::database::Person> listPerson,
                      std::vector<robobreizh::database::Object> listObject);
bool validateTranscriptActions(std::vector<std::string>& transcript);

std::vector<std::string> getIntent(std::string transcript);
bool isValidObject(std::string objName);
bool isValidPlace(std::string placeName);
}  // namespace generic
}  // namespace dialog
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_
