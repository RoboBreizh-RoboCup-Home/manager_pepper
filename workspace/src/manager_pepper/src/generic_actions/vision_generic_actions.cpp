// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

// NAOQI --> Service
#include <std_msgs/Empty.h>

#include <boost/thread/thread.hpp>

#include "database_model/object_model.hpp"
#include "database_model/location_model.hpp"
#include "database_model/person_model.hpp"
#include "generic_actions/vision_generic_actions.hpp"

#include "plan_high_level_actions/navigation_plan_actions.hpp"
#include "generic_actions/navigation_generic_actions.hpp"
#include "manager_utils.hpp"
#include "vision_utils.hpp"

using namespace std;

namespace robobreizh
{
namespace vision
{
namespace generic
{
bool findHostAndStoreFeaturesWithDistanceFilter(double distanceMax)
{
  return true;
}

/*******************************************************************/
bool waitForHuman()
{
  return true;
}

bool findObject(std::string objectName)
{
  return true;
}

/*******************************************************************/
bool WaitForHumanWavingHand()
{
  return true;
}

/*******************************************************************/
bool FindEmptySeat()
{
  return true;
}

/*******************************************************************/
bool isDoorOpened()  // TODO: What if door not found => Use Enum instead (Open, closed, NotFound)
{
  return true;
}

/*******************************************************************/
bool findHumanAndStoreFeatures(robobreizh::database::Person* person)
{
  return true;
}

bool findStoreObjectAtLocation(std::string objectName, std::string objectLocation)
{
  return true;
}

/*******************************************************************/
bool findStoreAllObjects()
{
  return true;
}
/*******************************************************************/

bool findAndLocateBag()
{
  return true;
}

bool findAndLocateCabDriver()
{
  return true;
}

std::string findAndLocateLastObjectPose()
{
  return "";
}

/*******************************************************************/
int findHumanAndStoreFeaturesWithDistanceFilter(double distanceMax)
{
  return 0;
}

int breakTheRules(double distanceMax)
{
  return 0;
}

}  // namespace generic
}  // namespace vision
}  // namespace robobreizh
