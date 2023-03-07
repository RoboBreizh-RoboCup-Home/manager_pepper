#ifndef _PNP_ROBOBREIZH_VISION_UTILS_
#define _PNP_ROBOBREIZH_VISION_UTILS_
#include "perception_pepper/Person.h"
#include "perception_pepper/Person_pose.h"
#include "database_model/database_utils.hpp"
#include <vector>
#include <string>

namespace robobreizh {
void objectMsgToObjectStruct(robobreizh::database::Object* object, perception_pepper::Object objectMsg,
                             geometry_msgs::Point coord);
void personMsgToPersonStruct(robobreizh::database::Person* person, perception_pepper::Person pers,
                             perception_pepper::Person_pose persPose, geometry_msgs::Point coord);
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
