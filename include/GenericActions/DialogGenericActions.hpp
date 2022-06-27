#ifndef _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_

#include <std_msgs/String.h>
#include <DatabaseModel/VisionModel.hpp>

namespace robobreizh
{
    namespace dialog
    {
        namespace generic
        {
            bool robotSpeech(std::string text);
            std::vector<std::string> ListenSpeech(std::string* sentence);
            std::string ListenSpeech(std::string param,std::string* sentence);
            std::vector<std::string> wavToIntent(std::string*);
            std::string wavToParsedParam(std::string param,std::string* sentence); 
            bool presentPerson(std::vector<Person> listPerson);
            bool presentPerson(Person person);
        }// namespace generic
    } // namespace dialog
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_DIALOG_GENERIC_ACTIONS_
