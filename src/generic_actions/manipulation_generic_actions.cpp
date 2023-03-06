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
  int argc = 2;
  char** argv = nullptr;
  argv = new char*[argc];
  argv[0]="--qi-url";
  argv[1]="tcp://localhost:9559";


  if (arm == "Right") {
    qi::ApplicationSession app(argc, argv);
    app.start();
    qi::SessionPtr session = app.session();
    ROS_INFO("Bending right arm session created");
    // move right arm using ALMotion service
    qi::AnyObject al_motion = session->service("ALMotion");
    ROS_INFO("Bending right arm session connected to almotion");
    std::vector<std::string> names = { "RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RElbowRoll", "RWristYaw" };
    std::vector<float> angles = { 0.0f, 0.8f, 1.55f, 1.1f, 1.5f };
    float fractionMaxSpeed = 0.1f;

    std::vector<float> stiffness= { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
    al_motion.call<void>("setStiffnesses", names ,stiffness);
    ROS_INFO("Bending right arm session set stiffness");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    al_motion.call<void>("setAngles", names, angles, fractionMaxSpeed);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    stiffness = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 
    al_motion.call<void>("setStiffnesses", names,stiffness);
  }

  return true;
}
}  // namespace generic
}  // namespace manipulation
}  // namespace robobreizh
