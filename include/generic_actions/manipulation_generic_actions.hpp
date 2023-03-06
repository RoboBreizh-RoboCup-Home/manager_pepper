#ifndef _PNP_ROBOBREIZH_MANIPULATION_GENERIC_ACTIONS_
#define _PNP_ROBOBREIZH_MANIPULATION_GENERIC_ACTIONS_

#include <std_msgs/String.h>

namespace robobreizh {
namespace manipulation {
namespace generic {
bool grabHandle(std::string object, std::string hand);  //* Instead on object in string type, may be more logical to use
                                                        // position instead *//
bool dropObject(std::string hand);  //* A variant with the position where we need to put the object may be pretty useful
                                    // IMHO *//
bool bendArm(std::string arm);
}  // namespace generic
}  // namespace manipulation
}  // namespace robobreizh

#endif  // _PNP_ROBOBREIZH_MANIPULATION_GENERIC_ACTIONS_