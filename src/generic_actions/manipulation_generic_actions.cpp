#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>
// #include <alproxies/almotionproxy.h>
#include <boost/thread/thread.hpp>
#include "generic_actions/manipulation_generic_actions.hpp"

// Aldebaran includes
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

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
bool bendArm(string arm) {
  int argc = 0;
  char** argv = nullptr;
  // argv = new char*[2];
  // argv[0] = "--qi-url";
  // argv[1] = "tcp://localhost:9559";

  qi::ApplicationSession app(argc, argv);
  qi::SessionPtr session = app.session();
  // move right arm using ALMotion service
  qi::AnyObject al_motion = session->service("ALMotion");
  // AL::ALValue names = AL::ALValue::array("RShoulderRoll", "RShoulderPitch", " RElbowYaw", "RElbowRoll", "RWristYaw");
  // AL::ALValue angles = AL::ALValue::array(0.0f, 0.8f, 1.55f, 1.1f, 1.5f);
  float fractionMaxSpeed = 0.1f;
  al_motion.call<void>("setStiffnesses", 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // session->service("ALMotion").call<void>("setAngles", names, angles, fractionMaxSpeed);
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // session->service("ALMotion").call<void>("setStiffnesses", names, AL::ALValue::array(0.0f, 0.0f, 0.0f, 0.0f, 0.0f));

  app.run();

  return true;
}
}  // namespace generic
}  // namespace manipulation
}  // namespace robobreizh
