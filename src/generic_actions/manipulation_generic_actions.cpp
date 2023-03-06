#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>
// #include <alproxies/almotionproxy.h>
#include <boost/thread/thread.hpp>

#include "generic_actions/manipulation_generic_actions.hpp"

using namespace std;

namespace robobreizh {
namespace manipulation {
namespace generic {
bool grabHandle(string object, string hand) {
  //* Instead on object in string type, may be more logical to use position instead *//
  return true;
}
bool dropObject(string hand) {
  //* A variant with the position where we need to put the object may be pretty useful IMHO *//
  return true;
}
bool bendArm(string arm, string direction) {
  // std::string robotIp = "127.0.0.1";
  // AL::ALMotionProxy motion(robotIp);

  // // Example showing how to set angles, using a fraction of max speed

  // AL::ALValue names = AL::ALValue::array("RShoulderRoll", "RShoulderPitch", " RElbowYaw", "RElbowRoll", "RWristYaw");
  // AL::ALValue angles = AL::ALValue::array(0.0f, 0.8f, 1.55f, 1.1f, 1.5f);
  // float fractionMaxSpeed = 0.1f;
  // motion.setStiffnesses(names, AL::ALValue::array(1.0f, 1.0f, 1.0f, 1.0f, 1.0f));
  // qi::os::sleep(1.0f);
  // motion.setAngles(names, angles, fractionMaxSpeed);
  // qi::os::sleep(1.0f);
  // motion.setStiffnesses(names, AL::ALValue::array(0.0f, 0.0f));

  // return true;
}
}  // namespace generic
}  // namespace manipulation
}  // namespace robobreizh
