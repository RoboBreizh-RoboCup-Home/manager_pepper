#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dialog_pepper/Msg.h>
//#include <robobreizh_demo_components/Person.h>

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
    string intent; 

    void intentCb(const std_msgs::String::ConstPtr& msg){
      intent = msg->data.c_str();
    }
    bool robotSpeech(string text)
    {
      ROS_INFO("Text to pronounce: %s", text.c_str());
      ros::NodeHandle nh;
      ros::ServiceClient client = nh.serviceClient<dialog_pepper::Msg>("/robobreizh/dialog_pepper/tts_srv");
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
      }
    }

    string ListenSpeech()
    {
      ros::NodeHandle nh;
      ros::ServiceClient client = nh.serviceClient<dialog_pepper::Action>("/robobreizh/dialog_pepper/sti_srv");
      dialog_pepper::Action srv;
      srv.request.start = true;
      if (client.call(srv))
      {
        ROS_INFO("Intent received: %s", srv.response.intent);
        return srv.response.intent;
      }
      else
      {
        ROS_INFO("Failed to call service sti_srv");
        return "";
      }
    }
} // namespace generic
} // namespace dialog
} // namespace robobreizh
