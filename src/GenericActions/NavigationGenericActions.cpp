#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/NavigationGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace navigation
{
namespace generic
{
bool moveTowardsObject(string objectName /** Or object position if you prefer**/)
{
    // We can for example use a ros service here
    return true;
}

bool moveTowardsPosition(float x, float y, float z)
{
    return true;
}

} // namespace generic
} // namespace navigation
} // namespace robobreizh