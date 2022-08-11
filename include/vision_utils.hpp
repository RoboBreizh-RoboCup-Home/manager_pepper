#include "perception_pepper/Person.h"
#include "perception_pepper/Person_pose.h"

namespace robobreizh
{
namespace database
{
void objectMsgToObjectStruct(robobreizh::database::Object* object, perception_pepper::Object objectMsg,
                             geometry_msgs::Point coord);
void personMsgToPersonStruct(robobreizh::database::Person* person, perception_pepper::Person pers,
                             perception_pepper::Person_pose persPose, geometry_msgs::Point coord);
vector<std_msgs::String> fillTabMsg(vector<std::string> detections);
}  // namespace database
}  // namespace robobreizh