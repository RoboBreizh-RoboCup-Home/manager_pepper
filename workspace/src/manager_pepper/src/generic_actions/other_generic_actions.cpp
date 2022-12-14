#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>
#include <vector>
#include <string>
#include <boost/thread/thread.hpp>

#include "generic_actions/other_generic_actions.hpp"

using namespace std;

namespace robobreizh
{
namespace other
{
namespace generic
{
bool waitForGoSignal()
{
  return true;
}

}  // namespace generic
}  // namespace other
}  // namespace robobreizh
