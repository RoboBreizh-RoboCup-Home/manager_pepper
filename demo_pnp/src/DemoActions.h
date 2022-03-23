#ifndef _DEMO_PNP_ACTIONS_
#define _DEMO_PNP_ACTIONS_

#include <std_msgs/String.h>
#include "DemoActions.h"

// robotname as external variable (defined in MyActions.cpp)
extern std::string robotname;

// Action implementation

void ainit(std::string params, bool* run);
void acheck_color(std::string params, bool* run);
/*void gotopose(string params, bool *run);
void home(string params, bool *run);
void wave(string params, bool *run);
void turn360(string params, bool *run);
void sense1(string params, bool *run);

// Condition implementation
int closeToHomeCond(string params);*/

#endif

