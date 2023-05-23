#ifndef _PNP_ROBOBREIZH_VISION_UTILS_
#define _PNP_ROBOBREIZH_VISION_UTILS_
#include "robobreizh_msgs/Person.h"
#include "robobreizh_msgs/PersonPose.h"
#include "database_model/database_utils.hpp"
#include <vector>
#include <string>

namespace robobreizh {
void objectMsgToObjectStruct(robobreizh::database::Object* object, robobreizh_msgs::Object objectMsg,
                             geometry_msgs::Point coord);
void personMsgToPersonStruct(robobreizh::database::Person* person, robobreizh_msgs::Person pers,
                             geometry_msgs::Point coord);
void personMsgToPersonPoseStruct(robobreizh::database::Person* person, robobreizh_msgs::Person pers,
                                 robobreizh_msgs::PersonPose persPose, geometry_msgs::Point coord);
std::vector<std_msgs::String> fillTabMsg(std::vector<std::string> detections);
int isInForbiddenRoom(float x, float y);
bool addPersonToDatabase(robobreizh::database::Person person);
std::string findObjectCategory(std::string object);
std::string findObjectRange(std::string object, geometry_msgs::Point32 coord);
float convertOdomToBaseFootprint(float odomx, float odomy, float odomz);
geometry_msgs::Point convertOdomToMap(float odomx, float odomy, float odomz);
bool isInRadius(float x1, float y1, float z1, float x2, float y2, float z2, float epsilon);
bool addObjectToDatabase(robobreizh::database::Object obj);
}  // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_VISION_UTILS_
