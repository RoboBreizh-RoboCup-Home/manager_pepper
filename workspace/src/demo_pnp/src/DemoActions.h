#ifndef _DEMO_PNP_ACTIONS_
#define _DEMO_PNP_ACTIONS_

#include <std_msgs/String.h>
#include "DemoActions.h"

// robotname as external variable (defined in MyActions.cpp)
extern std::string robotname;

// Action implementation

void ainit(std::string params, bool* run);
void acheck_color(std::string params, bool* run);
void ademo_ros_service(std::string params, bool* run);

// Condition implementation
//int closeToHomeCond(string params);

#endif

