#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <boost/algorithm/string.hpp>
/*#include <dialog_pepper/Msg.h>
#include <dialog_pepper/Wti.h>
#include <dialog_pepper/WavString.h>
#include <dialog_pepper/Speech_processing.h>*/

#include <boost/thread/thread.hpp>

#include "GenericActions/DialogGenericActions.hpp"

using namespace std;

string intent;

namespace robobreizh
{
namespace dialog
{
namespace generic
{

    bool robotSpeech(string text)
    {
      ROS_INFO("Text to pronounce: %s", text.c_str());
      /*ros::NodeHandle nh;
      ros::ServiceClient client = nh.serviceClient<dialog_pepper::Msg>("/robobreizh/dialog_pepper/text_to_speech");
      dialog_pepper::Msg srv;
      srv.request.sentence = text;

      if (client.call(srv))
      {
        ROS_INFO("TTS success: %d", srv.response.success);
        return true;
      }
      else
      {
        ROS_INFO("Failed to call service pepper_speech");
        return false;
      }*/
      return true;
    }

    std::vector<string> wavToIntent(){
        std::vector<string> intent; 
        /*ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<dialog_pepper::Wti>("/robobreizh/dialog_pepper/wav_to_intent");
        dialog_pepper::Wti srv;
        srv.request.start= true;
        if (client.call(srv))
        {
            for (int i = 0 ; i < srv.response.intent.size(); i++ ){
                ROS_INFO("Intent received: %s", srv.response.intent[i].c_str());
                intent.push_back(srv.response.intent[i].c_str());
            }
        }
        else
        {
            ROS_INFO("Failed to call service wav_to_intent");
        }*/
        // TODO: Parse intent and insert it on database
        // TODO: Update total number of actions parameter
        return intent;
    } 

    std::vector<string> ListenSpeech()
    {
        /*ros::NodeHandle nh;

        boost::shared_ptr<dialog_pepper::Speech_processing const> isWritten;
        isWritten= ros::topic::waitForMessage<dialog_pepper::Speech_processing>("/robobreizh/dialog_pepper/speech_to_wav",nh);

        if (isWritten)
        {
            ROS_INFO("File written");
            std::vector<string> intent; 
            intent = wavToIntent();
            return intent;
        }
        else
        {
            ROS_INFO("Failed to call service sti_srv");
            std::vector<string> intent; 
            return intent;
        }*/

        std::vector<string> intent;
        return intent;
    }

    std::string wavToParsedParam(std::string param){
        std::string param_res;
        /*ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<dialog_pepper::WavString>("/robobreizh/dialog_pepper/parser_from_file_srv");
        dialog_pepper::WavString srv;
        srv.request.file_name = param;
        if (client.call(srv))
        {
            param_res = srv.response.result;
            ROS_INFO("Typed parsed: %s, res: %s", param.c_str(),param_res.c_str());
        }
        else
        {
            ROS_INFO("Failed to call service wav_to_intent");
        }*/
        return param_res;
    }

    std::string ListenSpeech(std::string param){
        /*ros::NodeHandle nh;

        boost::shared_ptr<dialog_pepper::Speech_processing const> isWritten;
        isWritten= ros::topic::waitForMessage<dialog_pepper::Speech_processing>("/robobreizh/dialog_pepper/speech_to_wav",nh);

        if (isWritten)
        {
            ROS_INFO("File written");
            std::string type_res; 
            type_res = wavToParsedParam(param);
            return type_res;
        }
        else
        {
            ROS_INFO("Failed to call service sti_srv");
            std::string type_res; 
            return type_res;
        }*/

        std::string type_res; 
        return type_res;

    } 

    bool presentPerson(Person person){
        std::string sentence = " Here is " + person.name + ". ";
        std::string pronoun;
        if (person.gender.compare("H")){
            pronoun = "He";
            sentence += pronoun + " is a guy."; 
        } else {
            pronoun = "She";
            sentence += pronoun + " is a girl."; 
        }

        sentence += pronoun + " likes drinking " + person.favorite_drink + ". ";
        if (!person.age.empty()){
            sentence += pronoun + " is between " + person.age+ " years old. ";
        }
        if (!person.cloth_color.empty()){
            sentence += pronoun + " wears " + person.cloth_color+ " cloth. ";
        }
        if (!person.skin_color.empty()){
            sentence += pronoun + " skin is " + person.skin_color;
        }
        std::cout << sentence << std::endl;
        return dialog::generic::robotSpeech(sentence);
    }

    bool presentPerson(std::vector<Person> listPerson){
        bool serviceWentThrough = true;

        for (auto &person: listPerson){
            serviceWentThrough = serviceWentThrough && presentPerson(person);
        }
        return serviceWentThrough;
    }

    string cleanString(string &str) 
    {
        std::replace(str.begin(), str.end(), '\'', ' '); 
        std::replace(str.begin(), str.end(), '{', ' '); 
        std::replace(str.begin(), str.end(), '}', ' ');
        return str;
    }


    database::GPSRAction getActionFromString(string &str)
    { 
        database::GPSRAction gpsrAction;
        std::vector<std::string> out;

        string cleanStr = cleanString(str);             //remove unnecessary characters 
        
        boost::split(out, cleanStr, boost::is_any_of(","));   // split string by ','
        
        for (auto &s: out)                                     //construct gpsrAction by extracting features and their values
        {
        std::vector<std::string> tokens;
            boost::split(tokens, s, boost::is_any_of(":"));   
            boost::algorithm::trim(tokens[0]);                 //removing white spaces
            boost::algorithm::trim(tokens[1]);          
            
            if(tokens[0] == "intent")           
                gpsrAction.intent = tokens[1];
                
            if(tokens[0] =="destination")
                gpsrAction.destination = tokens[1];

                
            if(tokens[0] == "person") 
                gpsrAction.person = tokens[1];
            
            
            if(tokens[0] == "object")
                gpsrAction.object_item = tokens[1];
            
                
            if(tokens[0] =="who") 
                gpsrAction.who = tokens[1];
            
                
            if(tokens[0] =="what") 
                gpsrAction.what = tokens[1];
            
        }
        return gpsrAction;
    }
} // namespace generic
} // namespace dialog
} // namespace robobreizh
