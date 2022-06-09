#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <dialog_pepper/Msg.h>
//#include <dialog_pepper/Wti.h>
//#include <dialog_pepper/Speech_processing.h>

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
        return intent;
    } 

    std::vector<string> ListenSpeech()
    {
        std::vector<string> intent; 
        /*ros::NodeHandle nh;

        boost::shared_ptr<dialog_pepper::Speech_processing const> isWritten;
        std::cout << "the listening node is about to wait" << std::endl;

        isWritten= ros::topic::waitForMessage<dialog_pepper::Speech_processing>("/robobreizh/dialog_pepper/speech_to_wav",nh);

        std::cout << "the listening node is finished waiting" << std::endl;

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

        return intent;
    }
} // namespace generic
} // namespace dialog
} // namespace robobreizh
