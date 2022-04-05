#ifndef _DEMO_PNP_ACTIONS_
#define _DEMO_PNP_ACTIONS_

#include <std_msgs/String.h>

// robotname as external variable (defined in MyActions.cpp)
extern std::string robotname;

// Action implementation

void ainit(std::string params, bool* run);
void await_for_go_signal(std::string params, bool* run);
void agreetings(std::string params, bool* run);
void await_for_human(std::string params, bool* run);

// Condition implementation
//int closeToHomeCond(string params);

#endif

