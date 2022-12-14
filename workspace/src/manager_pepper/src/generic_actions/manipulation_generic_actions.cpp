#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

#include <boost/thread/thread.hpp>

#include "generic_actions/manipulation_generic_actions.hpp"

using namespace std;

namespace robobreizh
{
namespace manipulation
{
namespace generic
{
bool grabHandle(string object, string hand)
{
  //* Instead on object in string type, may be more logical to use position instead *//
  return true;
}
bool dropObject(string hand)
{
  //* A variant with the position where we need to put the object may be pretty useful IMHO *//
  return true;
}
}  // namespace generic
}  // namespace manipulation
}  // namespace robobreizh
