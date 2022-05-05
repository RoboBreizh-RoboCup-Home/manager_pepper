#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/DialogGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace dialog
{
namespace generic
{
    bool robotSpeech(string text)
    {
      ROS_INFO("Text to pronounce: %s", text.c_str());
      return true;
      /*ros::NodeHandle nh;
      os::ServiceClient client = nh.serviceClient<robobreizh_demo_components::PepperSpeech>("pepper_speech");
      robobreizh_demo_components::PepperSpeech srv;

      srv.request.Text = text;

      if (client.call(srv))
      {
        // TODO Add other use case if speech fails inside server (we already have this information)
        return true;
      }
      else
      {
        ROS_INFO("Failed to call service pepper_speech");
        return false;
      }*/
    }

    string ListenSpeech()
    {
      string transcript = "Empty";
      return transcript;
    }
} // namespace generic
} // namespace dialog
} // namespace robobreizh