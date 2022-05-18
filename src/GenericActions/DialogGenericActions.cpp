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
      ros::ServiceClient client = nh.serviceClient<dialog_pepper::Msg>("/robobreizh/tts_srv");
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
      ros::Subscriber dialog_node = nh.subscribe("/robobreizh/dialog",1000,&intentCb);
      ros::spinOnce();
      return intent;
    }
} // namespace generic
} // namespace dialog
} // namespace robobreizh
