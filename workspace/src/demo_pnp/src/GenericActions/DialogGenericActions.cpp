#include <ros/ros.h>
#include <std_msgs/String.h>
#include <robobreizh_demo_components/PepperSpeech.h>
#include <robobreizh_demo_components/Person.h>

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
      ros::NodeHandle nh;
      ros::ServiceClient client = nh.serviceClient<robobreizh_demo_components::PepperSpeech>("pepper_speech");
      robobreizh_demo_components::PepperSpeech srv;

      srv.request.Text = text;

      if (client.call(srv))
      {
        // TODO Add other use case if speech fails inside server (we already have this information)
        return true;
      }
      else
      {
        cout << "Failed to call service pepper_speech" << endl;
        return false;
      }
    }
} // namespace generic
} // namespace dialog
} // namespace robobreizh